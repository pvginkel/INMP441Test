#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include "driver/i2s_std.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include "secrets.h"

#define SAMPLE_RATE 16000
#define CHUNK_MS 20
#define BIT_WIDTH 32

#define DATA_PIN 1
#define SCK_PIN 2
#define WS_PIN 4

#define RECEIVER_PORT 55581
#define RECEIVER_IP "192.168.178.190"

#define WIFI_SSID "Thing-Fish"

#define __CONCAT3(a, b, c) a##b##c
#define CONCAT3(a, b, c) __CONCAT3(a, b, c)

#define DATA_BIT_WIDTH CONCAT3(I2S_DATA_BIT_WIDTH_, BIT_WIDTH, BIT)
#define SLOT_BIT_WIDTH CONCAT3(I2S_SLOT_BIT_WIDTH_, BIT_WIDTH, BIT)

#define CHUNK_LEN ((BIT_WIDTH / 8) * ((SAMPLE_RATE * CHUNK_MS) / 1000))

static const char *TAG = "main";

i2s_chan_handle_t chan;
struct sockaddr_in dest_addr;
int sock;
EventGroupHandle_t wifi_event_group;
SemaphoreHandle_t sem;

void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Connecting to AP");

        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        auto event = (wifi_event_sta_disconnected_t *)event_data;

        ESP_LOGW(TAG, "Disconnected from AP, reason %d", event->reason);

        esp_restart();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        auto event = (ip_event_got_ip_t *)event_data;

        ESP_LOGI(TAG, "Got ip:" IPSTR, IP2STR(&event->ip_info.ip));

        xSemaphoreGive(sem);
    }
}

void setup_flash() {
    ESP_LOGI(TAG, "Setting up flash");

    auto ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void setup_wifi() {
    ESP_LOGI(TAG, "Setting up WiFi");

    sem = xSemaphoreCreateBinary();

    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, nullptr,
                                                        &instance_any_id));

    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, nullptr,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta =
            {
                .ssid = WIFI_SSID,
                .password = WIFI_PASSWORD,

                // Authmode threshold resets to WPA2 as default if password
                // matches WPA2 standards (pasword len => 8). If you want to
                // connect the device to deprecated WEP/WPA networks, Please set
                // the threshold value to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and
                // set the password with length and format matching to
                // WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.

                .threshold =
                    {
                        .authmode = WIFI_AUTH_WPA2_PSK,
                    },
                .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
                .sae_h2e_identifier = "",
            },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    xSemaphoreTake(sem, portMAX_DELAY);
}

void setup_socket() {
    ESP_LOGI(TAG, "Setting up socket");

    dest_addr.sin_addr.s_addr = inet_addr(RECEIVER_IP);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(RECEIVER_PORT);

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    assert(sock >= 0);
}

void setup_i2s() {
    ESP_LOGI(TAG, "Setting up I2S recording");

    i2s_chan_config_t chan_config = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_config, NULL, &chan));

    // See https://www.esp32.com/viewtopic.php?t=32328.

    i2s_std_config_t rx_std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg =
            {
                .data_bit_width = DATA_BIT_WIDTH,
                .slot_bit_width = SLOT_BIT_WIDTH,
                .slot_mode = I2S_SLOT_MODE_STEREO,
                .slot_mask = I2S_STD_SLOT_BOTH,
                .ws_width = BIT_WIDTH,
                .ws_pol = false,
                .bit_shift = false,
                .left_align = true,
                .big_endian = false,
                .bit_order_lsb = false,
            },
        .gpio_cfg =
            {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = (gpio_num_t)SCK_PIN,
                .ws = (gpio_num_t)WS_PIN,
                .dout = I2S_GPIO_UNUSED,
                .din = (gpio_num_t)DATA_PIN,
                .invert_flags =
                    {
                        .mclk_inv = false,
                        .bclk_inv = false,
                        .ws_inv = false,
                    },
            },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(chan, &rx_std_cfg));
}

extern "C" void app_main(void) {
    setup_flash();
    setup_wifi();
    setup_socket();
    setup_i2s();

    ESP_LOGI(TAG, "Recording audio");

    const auto buffer_len = CHUNK_LEN * 4;
    const auto buffer = (uint8_t *)malloc(buffer_len);
    const auto target_buffer_len = buffer_len;
    const auto target_buffer = (uint8_t *)malloc(target_buffer_len);

    ESP_ERROR_CHECK(i2s_channel_enable(chan));

    while (true) {
        size_t read;
        ESP_ERROR_CHECK(i2s_channel_read(chan, buffer, buffer_len, &read, portMAX_DELAY));

        auto source = (int32_t *)buffer;
        auto target = (int32_t *)target_buffer;
        auto samples = read / 4;

        for (int i = 0; i < samples; i++) {
            auto sample = *source++;
            sample = (sample >> 7) << 8;
            *target++ = sample;
        }

        int err = sendto(sock, target_buffer, read, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        assert(err >= 0);
    }
}
