#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <algorithm>

#include "driver/i2s_std.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
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
#define BIT_WIDTH 32

#define DATA_PIN 1
#define SCK_PIN 2
#define WS_PIN 4

#define RECEIVER_PORT 55581
#define RECEIVER_IP "192.168.178.190"

#define GAIN_BITS 2

#define WIFI_SSID "Thing-Fish"

#define esp_get_millis() uint32_t(esp_timer_get_time() / 1000ull)

#define __CONCAT3(a, b, c) a##b##c
#define CONCAT3(a, b, c) __CONCAT3(a, b, c)

#define DATA_BIT_WIDTH CONCAT3(I2S_DATA_BIT_WIDTH_, BIT_WIDTH, BIT)
#define SLOT_BIT_WIDTH CONCAT3(I2S_SLOT_BIT_WIDTH_, BIT_WIDTH, BIT)

static const char *TAG = "main";

i2s_chan_handle_t chan;
struct sockaddr_in dest_addr;
int sock;
EventGroupHandle_t wifi_event_group;
SemaphoreHandle_t sem;
srmodel_list_t *models;
esp_afe_sr_iface_t *afe_handle;
esp_afe_sr_data_t *afe_data;

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
                .slot_mode = I2S_SLOT_MODE_MONO,
                .slot_mask = I2S_STD_SLOT_LEFT,
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

void setup_afe() {
    models = esp_srmodel_init("model");

    auto afe_config = afe_config_init("MR", models, AFE_TYPE_VC, AFE_MODE_LOW_COST);

    ESP_LOGI(TAG, "aec_init %d", (int)afe_config->aec_init);
    ESP_LOGI(TAG, "aec_mode %d", (int)afe_config->aec_mode);
    ESP_LOGI(TAG, "aec_filter_length %d", (int)afe_config->aec_filter_length);
    ESP_LOGI(TAG, "se_init %d", (int)afe_config->se_init);
    ESP_LOGI(TAG, "ns_init %d", (int)afe_config->ns_init);
    if (afe_config->ns_model_name) {
        ESP_LOGI(TAG, "ns_model_name %s", afe_config->ns_model_name);
    }
    ESP_LOGI(TAG, "afe_ns_mode %d", (int)afe_config->afe_ns_mode);
    ESP_LOGI(TAG, "vad_init %d", (int)afe_config->vad_init);
    ESP_LOGI(TAG, "vad_mode %d", (int)afe_config->vad_mode);
    if (afe_config->vad_model_name) {
        ESP_LOGI(TAG, "vad_model_name %s", afe_config->vad_model_name);
    }
    ESP_LOGI(TAG, "vad_min_speech_ms %d", (int)afe_config->vad_min_speech_ms);
    ESP_LOGI(TAG, "vad_min_noise_ms %d", (int)afe_config->vad_min_noise_ms);
    ESP_LOGI(TAG, "vad_delay_ms %d", (int)afe_config->vad_delay_ms);
    ESP_LOGI(TAG, "vad_mute_playback %d", (int)afe_config->vad_mute_playback);
    ESP_LOGI(TAG, "vad_enable_channel_trigger %d", (int)afe_config->vad_enable_channel_trigger);
    ESP_LOGI(TAG, "wakenet_init %d", (int)afe_config->wakenet_init);
    if (afe_config->wakenet_model_name) {
        ESP_LOGI(TAG, "wakenet_model_name %s", afe_config->wakenet_model_name);
    }
    if (afe_config->wakenet_model_name_2) {
        ESP_LOGI(TAG, "wakenet_model_name_2 %s", afe_config->wakenet_model_name_2);
    }
    ESP_LOGI(TAG, "wakenet_mode %d", (int)afe_config->wakenet_mode);
    ESP_LOGI(TAG, "agc_init %d", (int)afe_config->agc_init);
    ESP_LOGI(TAG, "agc_mode %d", (int)afe_config->agc_mode);
    ESP_LOGI(TAG, "agc_compression_gain_db %d", (int)afe_config->agc_compression_gain_db);
    ESP_LOGI(TAG, "agc_target_level_dbfs %d", (int)afe_config->agc_target_level_dbfs);
    // ESP_LOGI(TAG, "pcm_config %d", (int)afe_config->pcm_config);
    ESP_LOGI(TAG, "afe_mode %d", (int)afe_config->afe_mode);
    ESP_LOGI(TAG, "afe_type %d", (int)afe_config->afe_type);
    ESP_LOGI(TAG, "afe_perferred_core %d", (int)afe_config->afe_perferred_core);
    ESP_LOGI(TAG, "afe_perferred_priority %d", (int)afe_config->afe_perferred_priority);
    ESP_LOGI(TAG, "afe_ringbuf_size %d", (int)afe_config->afe_ringbuf_size);
    ESP_LOGI(TAG, "memory_alloc_mode %d", (int)afe_config->memory_alloc_mode);
    ESP_LOGI(TAG, "afe_linear_gain %f", afe_config->afe_linear_gain);
    ESP_LOGI(TAG, "debug_init %d", (int)afe_config->debug_init);
    ESP_LOGI(TAG, "fixed_first_channel %d", (int)afe_config->fixed_first_channel);

    //
    // Assuming 240 MHz, the CPU usage of the feed task is as follows:
    //
    // - None of the modules enabled: <1 %
    // - Acoustic Echo Cancellation: 45 %
    // - Noise Suppression: 16 %
    // - Automatic Gain Control: 2 %
    //

    /********** AEC(Acoustic Echo Cancellation) **********/
    // Whether to init aec
    assert(afe_config->aec_init == true);
    // The mode of aec, AEC_MODE_SR_LOW_COST or AEC_MODE_SR_HIGH_PERF
    // afe_config->aec_mode;
    // The filter length of aec
    // afe_config->aec_filter_length;

    /********** SE(Speech Enhancement, microphone array processing) **********/
    // Whether to init se
    assert(afe_config->se_init == false);

    /********** NS(Noise Suppression) **********/
    // Whether to init ns
    assert(afe_config->ns_init == true);
    // Model name of ns
    // afe_config->ns_model_name;
    // Model mode of ns
    // afe_config->afe_ns_mode;

    /********** VAD(Voice Activity Detection) **********/
    // Whether to init vad
    afe_config->vad_init = false;
    // The value can be: VAD_MODE_0, VAD_MODE_1, VAD_MODE_2, VAD_MODE_3, VAD_MODE_4
    // afe_config->vad_mode;
    // The model name of vad, If it is null, WebRTC VAD will be used.
    // afe_config->vad_model_name;
    // The minimum duration of speech in ms. It should be bigger than 32 ms, default: 128 ms
    // afe_config->vad_min_speech_ms;
    // The minimum duration of noise or silence in ms. It should be bigger than 64 ms, default:
    // 1000 ms
    // afe_config->vad_min_noise_ms;
    // The delay of the first speech frame in ms, default: 128 ms
    // If you find vad cache can not cover all speech, please increase this value.
    // afe_config->vad_delay_ms;
    // If true, the playback will be muted for vad detection. default: false
    // afe_config->vad_mute_playback;
    // If true, the vad will be used to choose the channel id. default: false
    // afe_config->vad_enable_channel_trigger;

    /********** WakeNet(Wake Word Engine) **********/
    assert(afe_config->wakenet_init == false);
    // The model name of wakenet 1
    // afe_config->wakenet_model_name;
    // The model name of wakenet 2 if has wakenet 2
    // afe_config->wakenet_model_name_2;
    // The mode of wakenet
    // afe_config->wakenet_mode;

    /********** AGC(Automatic Gain Control) **********/
    // Whether to init agc
    afe_config->agc_init = true;
    // The AGC mode for ASR. and the gain generated by AGC acts on the audio after far linear gain.
    assert(afe_config->agc_mode == AFE_AGC_MODE_WEBRTC);
    // Compression gain in dB (default 9)
    assert(afe_config->agc_compression_gain_db == 9);
    // Target level in -dBfs of envelope (default -3)
    assert(afe_config->agc_target_level_dbfs == 3);

    /********** General AFE(Audio Front End) parameter **********/
    // Config the channel num of original data which is fed to the afe feed function.
    // afe_config->pcm_config;
    // The mode of afe， AFE_MODE_LOW_COST or AFE_MODE_HIGH_PERF
    // afe_config->afe_mode;
    // The mode of afe， AFE_MODE_LOW_COST or AFE_MODE_HIGH_PERF
    // afe_config->afe_type;
    // The preferred core of afe se task, which is created in afe_create function.
    afe_config->afe_perferred_core = 1;
    // afe_config->afe_perferred_core;
    // The preferred priority of afe se task, which is created in afe_create function.
    // afe_config->afe_perferred_priority;
    // The ring buffer size: the number of frame data in ring buffer.
    // afe_config->afe_ringbuf_size;
    // The memory alloc mode for afe. From Internal RAM or PSRAM
    // afe_config->memory_alloc_mode;
    // The linear gain for afe output the value should be in [0.1, 10.0]. This value acts directly on the output
    // amplitude: out_linear_gain * amplitude.
    // afe_config->afe_linear_gain;
    // afe_config->debug_init;
    // If true, the channel after first wake-up is fixed to raw data of microphone
    // otherwise, select channel number by wakenet
    // afe_config->fixed_first_channel;

    afe_handle = esp_afe_handle_from_config(afe_config);
    afe_data = afe_handle->create_from_config(afe_config);
    afe_config_free(afe_config);
}

int16_t parse_sample(int32_t raw_sample) {
    // The IMNP441 outputs the following data:
    //
    // Slot bit:  0  1  2  3  .. 23 24 25 26 .. 31
    // Data bit:     0  1  2  .. 22 23
    //               MSB      ..   LSB
    //
    // We want to truncate the sample to 16 bits. Easiest is to just take bit 1
    // through 16. However, by truncating a few of the upper bytes, we can implement gain
    // early on in processing the sample without loosing too much fidelity. GAIN is
    // the number of MSB bits we truncate. In practical terms the gain is 2 ^ GAIN.
    //
    // Assuming GAIN is 2, we want to get these bits:
    //
    // Slot bit:    0  1  2  3  4  .. 17 18 19 20 .. 23 24 25 26 .. 31
    // Data bit:       0  1  2  3  .. 16 17 18 19 .. 22 23
    // Target bit:           0  1  .. 14 15
    //                 MSB      ..   LSB
    //
    // What we want to do is shift away slot bits 19 through 31, i.e. 13 bits.

    return (int16_t)(raw_sample >> (11 + GAIN_BITS));
}

void forward_task(void *param) {
    while (true) {
        auto res = afe_handle->fetch_with_delay(afe_data, portMAX_DELAY);
        if (!res) {
            ESP_LOGE(TAG, "Failed to fetch");
            esp_restart();
        }
        ESP_ERROR_CHECK(res->ret_value);

        int err = sendto(sock, res->data, res->data_size, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        assert(err >= 0);
    }
}

void feed_task(void *param) {
    ESP_LOGI(TAG, "Recording audio");

    const auto feed_chunksize = afe_handle->get_feed_chunksize(afe_data);
    const auto feed_nch = afe_handle->get_feed_channel_num(afe_data);
    assert(feed_nch == 2);
    const auto feed_buffer_len = feed_chunksize * feed_nch * sizeof(int16_t);
    const auto feed_buffer = (int16_t *)malloc(feed_buffer_len);
    assert(feed_buffer);
    size_t feed_buffer_offset = 0;

    // We keep the sample buffer equal to the feed buffer to keep latency down.
    const auto buffer_len = feed_chunksize * 1 /* mono */ * sizeof(int32_t);
    const auto buffer = malloc(buffer_len);
    assert(buffer);

    ESP_LOGI(TAG, "buffer_len %d feed_chunksize %d feed_nch %d feed_buffer_len %d", (int)buffer_len,
             (int)feed_chunksize, (int)feed_nch, (int)feed_buffer_len);

    ESP_ERROR_CHECK(i2s_channel_enable(chan));

    while (true) {
        size_t read;
        ESP_ERROR_CHECK(i2s_channel_read(chan, buffer, (feed_chunksize * 4), &read, portMAX_DELAY));

        auto source = (int32_t *)buffer;
        auto samples = read / sizeof(int32_t);

        for (int i = 0; i < samples; i += 1) {
            feed_buffer[feed_buffer_offset++] = parse_sample(source[i]);  // Left mic.
            feed_buffer[feed_buffer_offset++] = 0;                        // Playback/reference channel.

            if (feed_buffer_offset * sizeof(int16_t) >= feed_buffer_len) {
                afe_handle->feed(afe_data, feed_buffer);
                feed_buffer_offset = 0;
            }
        }
    }
}

extern "C" void app_main(void) {
    setup_flash();
    setup_wifi();
    setup_socket();
    setup_i2s();
    setup_afe();

    ESP_LOGI(TAG, "Starting tasks");

    xTaskCreate(forward_task, "forward_task", CONFIG_ESP_MAIN_TASK_STACK_SIZE, nullptr, 5, nullptr);
    xTaskCreate(feed_task, "feed_task", 8 * 1024, nullptr, 5, nullptr);

    static auto task_buffer = (char *)malloc(2000);

    while (true) {
        ESP_LOGI(TAG, "Performance statistics");

        vTaskGetRunTimeStats(task_buffer);
        printf(task_buffer);

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
