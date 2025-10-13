#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"
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

typedef struct
{
    float temp;
    float hum;
} AHT22Result;

static AHT22Result FAILRESULT = {-1.0f, -1.0f};

static AHT22Result new_aht22result(float temperature, float humidity)
{
    AHT22Result r = {temperature, humidity};
    return r;
}

static bool is_failed(AHT22Result *result)
{
    return result->hum <= 0.0f || result->temp <= 0.0f;
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
static volatile bool mqtt_failed = false;
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
        mqtt_failed = true; // エラーを検知
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
        printf("AHT20: Temp=%.1f°C  Hum=%.1f%%\n", tmp, hum);
        return new_aht22result(tmp, hum);
    }
    else
    {
        printf("AHT20 read failed (r=%d)\n", r);
    }
    // 取得失敗
    // -1度以下になることを考慮していない。埼玉だから問題なしか、、、
    return FAILRESULT;
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

    // Wi-Fi/LwIP 初期化（BG スレッドで動く）
    if (cyw43_arch_init())
    {
        printf("cyw43_arch_init failed\n");
        return -1;
    }
    // 省電力/LED初期化などは内部にお任せ
    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi SSID: %s\n", WIFI_SSID);
    int r = cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 30000);
    if (r)
    {
        printf("Wi-Fi connect failed: %d\n", r);
        return -1;
    }

    // LED 点灯で接続表示（任意）
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    printf("Wi-Fi connected.\n");

    client = mqtt_client_new();
    if (!client)
    {
        printf("mqtt client new failed\n");
        return -1;
    }

    ip_addr_t broker_addr;
    ipaddr_aton(MQTT_BROKER_IP, &broker_addr);

    struct mqtt_connect_client_info_t ci = {0};
    ci.client_id = MQTT_CLIENT_ID;
    ci.keep_alive = 30;
    ci.client_user = NULL;
    ci.client_pass = NULL;

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

    while (true)
    {
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
        err_t pe = mqtt_publish(client, MQTT_TOPIC, payload, strlen(payload), 0, 0, mqtt_pub_request_cb, NULL);
        printf("publish: %s (err=%d)\n", payload, pe);
        sleep_ms(1000);
    }

    mqtt_client_free(client);
    cyw43_arch_deinit();
    return 0;
}
