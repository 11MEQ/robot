#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/i2s.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_websocket_client.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"

// Wake Word / Skainet headers
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "model_path.h"

// ================= WiFi Configuration =================
#define WIFI_SSID "AZ"
#define WIFI_PASS "8/2*3-6/6=10HHa"
#define WEBSOCKET_URI  "ws://192.168.1.3:9000"

// Event group bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// ================= Definitions =================
static const char *TAG = "ALEXA_DETECT";
#define I2S_NUM_MIC (0)

// I2S Pin Config
#define I2S_BCK_MIC (2)
#define I2S_WS_MIC (42)
#define I2S_DI_MIC (41)

// Audio streaming config
#define AUDIO_BUFFER_SIZE 1024
#define MAX_STREAMING_DURATION_MS 10000  // Maximum 30 seconds (safety timeout)
#define SILENCE_TIMEOUT_MS 1200  // Stop after 2 seconds of silence

// Global Handles
static esp_afe_sr_iface_t *afe_handle = NULL;
static srmodel_list_t *models = NULL;
static volatile int task_flag = 1;
static volatile bool pause_wake_word = false;
static volatile bool is_streaming = false;

static esp_websocket_client_handle_t websocket_client = NULL;
static EventGroupHandle_t wifi_event_group;

// ================= WiFi Event Handler =================
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Retrying WiFi connection...");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// ================= WiFi Initialization =================
void wifi_init(void)
{
    wifi_event_group = xEventGroupCreate();
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi initialization finished.");
    
    // Wait for connection
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to WiFi");
    } else {
        ESP_LOGE(TAG, "Failed to connect to WiFi");
    }
}

// ================= WebSocket Event Handler =================
static void websocket_event_handler(void *handler_args, esp_event_base_t base, 
                                   int32_t event_id, void *event_data)
{
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
    
    switch (event_id) {
        case WEBSOCKET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "WebSocket Connected");
            break;
        case WEBSOCKET_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "WebSocket Disconnected");
            break;
        case WEBSOCKET_EVENT_DATA:
            ESP_LOGI(TAG, "WebSocket Data Received: %d bytes", data->data_len);
            break;
        case WEBSOCKET_EVENT_ERROR:
            ESP_LOGE(TAG, "WebSocket Error");
            break;
        default:
            break;
    }
}

// ================= WebSocket Initialization =================
void websocket_init(void)
{
    esp_websocket_client_config_t ws_cfg = {
        .uri = WEBSOCKET_URI,
        .buffer_size = 1024,
        // .buffer_size = 2048,
    };

    websocket_client = esp_websocket_client_init(&ws_cfg);
    esp_websocket_register_events(websocket_client, WEBSOCKET_EVENT_ANY, 
                                  websocket_event_handler, NULL);
    
    ESP_LOGI(TAG, "Connecting to WebSocket...");
    esp_websocket_client_start(websocket_client);
}

// ================= Helper Functions =================
int custom_get_feed_data(bool is_get_raw_channel, int16_t *buffer, int buffer_len)
{
    size_t bytes_read = 0;
    int samples_count = buffer_len / sizeof(int16_t);
    int32_t *i2s_buffer = (int32_t *)malloc(samples_count * sizeof(int32_t));

    if (i2s_buffer == NULL) return 0;

    esp_err_t ret = i2s_read(I2S_NUM_MIC, i2s_buffer, samples_count * sizeof(int32_t),
                             &bytes_read, portMAX_DELAY);

    if (ret != ESP_OK)
    {
        free(i2s_buffer);
        return 0;
    }

    int samples_read = bytes_read / sizeof(int32_t);
    for (int i = 0; i < samples_read; i++)
    {
        buffer[i] = (int16_t)(i2s_buffer[i] >> 14);
    }

    free(i2s_buffer);
    return samples_read * sizeof(int16_t);
}

// ================= Voice Activity Detection Helper =================
bool is_voice_active(int16_t *buffer, int samples)
{
    // Calculate RMS (Root Mean Square) energy
    long long sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += (long long)buffer[i] * buffer[i];
    }
    int rms = (int)sqrt(sum / samples);
    
    // Threshold for voice detection (adjust based on your environment)
    // Typical values: 100-500 for quiet, 500-1500 for normal, 1500+ for noisy
    const int VOICE_THRESHOLD = 300;
    
    return (rms > VOICE_THRESHOLD);
}

// ================= Audio Streaming Task =================
void stream_audio_task(void *arg)
{
    ESP_LOGI(TAG, "🎤 Starting audio streaming with Voice Activity Detection...");
    
    int16_t *audio_buffer = (int16_t *)malloc(AUDIO_BUFFER_SIZE);
    if (audio_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate audio buffer");
        vTaskDelete(NULL);
        return;
    }

    TickType_t start_time = xTaskGetTickCount();
    TickType_t last_voice_time = xTaskGetTickCount();
    TickType_t max_duration_ticks = pdMS_TO_TICKS(MAX_STREAMING_DURATION_MS);
    TickType_t silence_timeout_ticks = pdMS_TO_TICKS(SILENCE_TIMEOUT_MS);
    
    int samples_per_buffer = AUDIO_BUFFER_SIZE / sizeof(int16_t);
    bool voice_detected_once = false;

    while (is_streaming) {
        // Safety timeout: stop after maximum duration
        if ((xTaskGetTickCount() - start_time) > max_duration_ticks) {
            ESP_LOGI(TAG, "Maximum streaming duration reached");
            break;
        }

        // Read audio data
        int bytes_read = custom_get_feed_data(false, audio_buffer, AUDIO_BUFFER_SIZE);
        
        if (bytes_read > 0) {
            // Check for voice activity
            bool voice_active = is_voice_active(audio_buffer, samples_per_buffer);
            
            if (voice_active) {
                last_voice_time = xTaskGetTickCount();
                voice_detected_once = true;
                ESP_LOGD(TAG, "Voice detected");
            }
            
            // Stop streaming if:
            // 1. Voice was detected at least once (to avoid stopping immediately)
            // 2. No voice detected for SILENCE_TIMEOUT_MS
            if (voice_detected_once && 
                (xTaskGetTickCount() - last_voice_time) > silence_timeout_ticks) {
                ESP_LOGI(TAG, "Silence detected - stopping stream");
                break;
            }
            
            // Send audio data via WebSocket
            if (esp_websocket_client_is_connected(websocket_client)) {
                int sent = esp_websocket_client_send_bin(websocket_client, 
                                                         (char *)audio_buffer, 
                                                         bytes_read, 
                                                         portMAX_DELAY);
                if (sent < 0) {
                    ESP_LOGE(TAG, "Failed to send audio data");
                }
            } else {
                ESP_LOGW(TAG, "WebSocket not connected, skipping audio packet");
            }
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    is_streaming = false;
    free(audio_buffer);
    
    ESP_LOGI(TAG, "🛑 Audio streaming stopped");
    
    // Resume wake word detection
    pause_wake_word = false;
    
    vTaskDelete(NULL);
}

// ================= Tasks =================
void feed_task(void *arg)
{
    esp_afe_sr_data_t *afe_data = (esp_afe_sr_data_t *)arg;
    int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
    int feed_channel = 1;
    int16_t *i2s_buff = (int16_t *)malloc(audio_chunksize * sizeof(int16_t) * feed_channel);
    assert(i2s_buff);

    ESP_LOGI(TAG, "Feed task started");

    while (task_flag)
    {
        if (pause_wake_word)
        {
            vTaskDelay(50 / portTICK_PERIOD_MS);
            continue;
        }

        int ret = custom_get_feed_data(false, i2s_buff, audio_chunksize * sizeof(int16_t) * feed_channel);

        if (ret > 0)
        {
            afe_handle->feed(afe_data, i2s_buff);
        }
        else
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }

    if (i2s_buff)
        free(i2s_buff);
    vTaskDelete(NULL);
}

void detect_task(void *arg)
{
    esp_afe_sr_data_t *afe_data = (esp_afe_sr_data_t *)arg;

    ESP_LOGI(TAG, "Wake Word Detection Active");

    while (task_flag)
    {
        if (pause_wake_word)
        {
            vTaskDelay(50 / portTICK_PERIOD_MS);
            continue;
        }

        afe_fetch_result_t *res = afe_handle->fetch(afe_data);

        if (!res || res->ret_value == ESP_FAIL)
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }

        if (res->wakeup_state == WAKENET_DETECTED)
        {
            ESP_LOGI(TAG, "✅ WAKE WORD DETECTED!");
            
            // Pause wake word detection during streaming
            pause_wake_word = true;
            is_streaming = true;
            
            // Start audio streaming task
            xTaskCreatePinnedToCore(&stream_audio_task, "stream_audio", 
                                   8 * 1024, NULL, 5, NULL, 1);
        }
    }
    vTaskDelete(NULL);
}

// ================= Initialization =================
bool init_wake_word_detection(void)
{
    ESP_LOGI(TAG, "Initializing wake word...");

    char *partition_label = "model";
    models = esp_srmodel_init(partition_label);

    if (models == NULL)
    {
        ESP_LOGE(TAG, "Model partition missing or empty!");
        return false;
    }

    afe_handle = (esp_afe_sr_iface_t *)&ESP_AFE_SR_HANDLE;
    afe_config_t afe_config = AFE_CONFIG_DEFAULT();

    afe_config.wakenet_model_name = esp_srmodel_filter(models, ESP_WN_PREFIX, NULL);

    if (afe_config.wakenet_model_name == NULL)
    {
        ESP_LOGE(TAG, "No wake-net model found");
        return false;
    }

    afe_config.aec_init = false;
    afe_config.pcm_config.total_ch_num = 1;
    afe_config.pcm_config.mic_num = 1;
    afe_config.pcm_config.ref_num = 0;

    ESP_LOGI(TAG, "Loaded Model: %s", afe_config.wakenet_model_name);

    esp_afe_sr_data_t *afe_data = afe_handle->create_from_config(&afe_config);
    if (afe_data == NULL)
    {
        ESP_LOGE(TAG, "Failed to create AFE instance");
        return false;
    }

    xTaskCreatePinnedToCore(&detect_task, "detect", 8 * 1024, (void *)afe_data, 5, NULL, 1);
    xTaskCreatePinnedToCore(&feed_task, "feed", 8 * 1024, (void *)afe_data, 5, NULL, 0);

    return true;
}

// ================= Main =================
void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize WiFi
    wifi_init();
    
    // Initialize WebSocket
    websocket_init();

    // Basic I2S Init for INMP441
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate = 16000,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2,
        .dma_buf_count = 6,
        .dma_buf_len = 512,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0};

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK_MIC,
        .ws_io_num = I2S_WS_MIC,
        .data_out_num = -1,
        .data_in_num = I2S_DI_MIC};

    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_MIC, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_MIC, &pin_config));
    ESP_ERROR_CHECK(i2s_zero_dma_buffer(I2S_NUM_MIC));

    // Start Wake Word Detection
    init_wake_word_detection();
    
    ESP_LOGI(TAG, "System ready. Listening for wake word...");
}
