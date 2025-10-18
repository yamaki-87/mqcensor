#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/apps/mqtt.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include "wifi_config.h"

#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID "pico2w"
#define MQTT_TOPIC "pico2w/aht22"
#define DHT_PIN 17

static absolute_time_t last_ok; // 直近で「正常」だった時刻（リンク or MQTT OK）
#define WD_TIMEOUT_MS 8000      // WDT 8秒
#define DEADLINE_MS 300000      // 5分復帰しなければ最終手段
#define SAFE_REBOOTS 5          // 5連続再起動でセーフモード突入

static inline bool ms_passed(absolute_time_t t, uint32_t ms)
{
    return absolute_time_diff_us(t, get_absolute_time()) / 1000 > ms;
}

static void wd_init_and_bootloop_guard(bool *safe_mode_out)
{
    // 連続再起動回数を scratch レジスタに保存
    uint32_t cnt = watchdog_hw->scratch[0];
    if (watchdog_caused_reboot())
        cnt++;
    else
        cnt = 0;
    watchdog_hw->scratch[0] = cnt;

    *safe_mode_out = (cnt >= SAFE_REBOOTS);

    // 有効化（デバッガ接続時は一時停止 true）
    watchdog_enable(WD_TIMEOUT_MS, true);
}

static void wd_feed(void)
{
    watchdog_update();
}

static void request_reboot_now(const char *reason)
{
    printf("WDT reboot requested: %s\n", reason);
    sleep_ms(50);
    watchdog_reboot(0, 0, 0);
    while (1)
        tight_loop_contents();
}

typedef struct
{
    float temp;
    float hum;
} AHT22Result;

static const AHT22Result FAILRESULT = {-100.0f, -100.0f};

static AHT22Result new_aht22result(float temperature, float humidity)
{
    AHT22Result r = {temperature, humidity};
    return r;
}

static bool is_failed(AHT22Result *result)
{
    return result->hum == -100.0f || result->temp <= -100.0f;
}

static bool link_is_up(void)
{
    int st = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
    return st == CYW43_LINK_UP;
}

static bool wifi_connect(void)
{
    int r = cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 30000);

    return r == 0;
}

static void print_ip(void)
{
    struct netif *n = &cyw43_state.netif[0];
    const ip4_addr_t *ip = netif_ip4_addr(n);
    const ip4_addr_t *gw = netif_ip4_gw(n);
    const ip4_addr_t *msk = netif_ip4_netmask(n);

    // IPアドレスを文字列形式に変換して表示
    char ip_str[16];
    char gw_str[16];
    char mask_str[16];

    // IPアドレスを文字列に変換
    ip4addr_ntoa_r(ip, ip_str, sizeof(ip_str));
    ip4addr_ntoa_r(gw, gw_str, sizeof(gw_str));
    ip4addr_ntoa_r(msk, mask_str, sizeof(mask_str));
    printf("Pico STA IP=%s GW=%s MASK=%s\n",
           ip_str, gw_str, mask_str);
}

static mqtt_client_t *client;
static volatile bool mqtt_connected = false;
const int SUCCESS = 6;

static void mqtt_pub_request_cb(void *arg, err_t result)
{
    printf("MQTT publish result: %d\n", result);
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
    printf("MQTT connection status: %d\n", status);
    if (status == MQTT_CONNECT_ACCEPTED)
        mqtt_connected = true;
    else
        mqtt_connected = false; // エラーを検知
}

static AHT22Result read_aht20(void)
{
    uint8_t cmd[3] = {0xAC, 0x33, 0x00};
    i2c_write_timeout_us(i2c0, 0x38, cmd, 3, false, 3000);
    sleep_ms(80);

    uint8_t buf[6];
    int r = i2c_read_timeout_us(i2c0, 0x38, buf, 6, false, 3000);
    if (r == SUCCESS)
    {
        uint32_t raw_h = ((uint32_t)(buf[1]) << 12) | ((uint32_t)buf[2] << 4) | (buf[3] >> 4);
        uint32_t raw_t = (((uint32_t)buf[3] & 0x0F) << 16) | ((uint32_t)buf[4] << 8) | buf[5];
        float hum = (raw_h * 100.0f) / 1048576.0f;
        float tmp = (raw_t * 200.0f) / 1048576.0f - 50.0f;
        // printf("AHT20: Temp=%.1f°C  Hum=%.1f%%\n", tmp, hum);
        return new_aht22result(tmp, hum);
    }
    else
    {
        // printf("AHT20 read failed (r=%d)\n", r);
    }
    // 取得失敗
    // -1度以下になることを考慮していない。埼玉だから問題なしか、、、
    return FAILRESULT;
}

static bool wifi_mqtt_conn_init(ip_addr_t broker_addr, struct mqtt_connect_client_info_t ci)
{
    bool ok = wifi_connect(); // 既存の関数
    if (!ok)
    {
        printf("Wi-Fi connect failed at boot\n");
        return false;
    }
    cyw43_arch_lwip_begin();
    err_t err = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT, mqtt_connection_cb, NULL, &ci);
    cyw43_arch_lwip_end();
    if (err != ERR_OK)
    {
        printf("mqtt_client_connect err=%d\n", err);
        return false;
    }

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    return true;
}

static struct mqtt_connect_client_info_t create_mqtt_client(void)
{
    struct mqtt_connect_client_info_t ci = {0};
    ci.client_id = MQTT_CLIENT_ID;
    ci.will_msg = "offline";
    ci.keep_alive = 30;
    ci.will_qos = 1;
    ci.will_retain = 1;
    ci.client_user = NULL;
    ci.client_pass = NULL;

    return ci;
}

int main()
{
    stdio_init_all();
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(17, GPIO_FUNC_I2C);
    gpio_set_function(16, GPIO_FUNC_I2C);
    gpio_pull_up(17);
    gpio_pull_up(16);
    printf("I2C scan start\n");
    sleep_ms(1500);
    printf("Pico2W MQTT publisher start\n");

    bool safe_mode = false;
    wd_init_and_bootloop_guard(&safe_mode);
    // Wi-Fi/LwIP 初期化（BG スレッドで動く）
    if (cyw43_arch_init())
    {
        printf("cyw43_arch_init failed\n");
        return -1;
    }
    // 省電力/LED初期化などは内部にお任せ
    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi SSID: %s\n", WIFI_SSID);
    if (!safe_mode)
    {
        // wifiがつながるまで無限ループ
        while (!wifi_connect())
        {
            sleep_ms(2000);
        }
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    }
    else
    {
        // セーフモード：Wi-Fiを明示的に下げる（人が触れる状態を優先）
        cyw43_wifi_set_up(&cyw43_state, CYW43_ITF_STA, false, 0);
        printf("SAFE MODE: Wi-Fi disabled due to repeated reboots\n");
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    }

    printf("Wi-Fi connected.\n");

    client = mqtt_client_new();
    if (!client)
    {
        printf("mqtt client new failed\n");
        return -1;
    }

    ip_addr_t broker_addr;
    ipaddr_aton(MQTT_BROKER_IP, &broker_addr);

    struct mqtt_connect_client_info_t ci = create_mqtt_client();

    cyw43_arch_lwip_begin();
    err_t err = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT, mqtt_connection_cb, NULL, &ci);
    cyw43_arch_lwip_end();
    if (err != ERR_OK)
    {
        printf("mqtt_client_connect err=%d\n", err);
        return -1;
    }

    // ★ コールバックで接続完了を待つ（sleep固定はやめる）
    while (!mqtt_connected)
    {
        sleep_ms(10);
    }

    last_ok = get_absolute_time();
    while (true)
    {
        wd_feed();
        if (link_is_up() && mqtt_connected)
        {
            last_ok = get_absolute_time();
        }
        else
        {
            bool is_success = wifi_mqtt_conn_init(broker_addr, ci);
            if (!is_success)
            {
                sleep_ms(1000);
                continue;
            }
        }
        // 5分以上「リンクUP && MQTT接続」の状態に戻れない → 最終手段
        if (!safe_mode && ms_passed(last_ok, DEADLINE_MS))
        {
            request_reboot_now("no recovery >5min");
        }
        char payload[64];
        AHT22Result r = read_aht20();
        if (is_failed(&r))
        {
            snprintf(payload, sizeof(payload), "failed");
        }
        else
        {
            snprintf(payload, sizeof(payload), "Temp=%.1f°C Hum=%.1f%%", r.temp, r.hum);
        }
        cyw43_arch_lwip_begin();
        err_t pe = mqtt_publish(client, MQTT_TOPIC, payload, strlen(payload), 0, 0, mqtt_pub_request_cb, NULL);
        cyw43_arch_lwip_end();
        printf("publish: %s (err=%d)\n", payload, pe);
        sleep_ms(1000);
    }

    mqtt_client_free(client);
    cyw43_arch_deinit();
    return 0;
}
