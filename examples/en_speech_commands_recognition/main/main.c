#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h> // <--- FIXED: Essential for esp_afe_sr headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "esp_log.h"
#include "esp_err.h"

// Wake Word / Skainet headers
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "model_path.h"

// ================= Definitions (FIXED) =================
static const char *TAG = "ALEXA_DETECT"; // <--- FIXED: Defined TAG
#define I2S_NUM_MIC (0)                  // <--- FIXED: Defined I2S Port

// I2S Pin Config (Matches your hardware description)
#define I2S_BCK_MIC (2)
#define I2S_WS_MIC (42)
#define I2S_DI_MIC (41)

// Global Handles
static esp_afe_sr_iface_t *afe_handle = NULL;
static srmodel_list_t *models = NULL;
static volatile int task_flag = 1;
static volatile bool pause_wake_word = false;

// ================= Helper Functions =================

// Helper to read data from I2S
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
    // INMP441 is 24-bit left justified in 32-bit slot.
    // We usually shift right by 14 or 15 to get good 16-bit PCM volume.
    for (int i = 0; i < samples_read; i++)
    {
        buffer[i] = (int16_t)(i2s_buffer[i] >> 14);
    }

    free(i2s_buffer);
    return samples_read * sizeof(int16_t);
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
            // Add your logic here (e.g., set a flag, turn on LED)
        }
    }
    vTaskDelete(NULL);
}

// ================= Initialization =================

bool init_wake_word_detection(void)
{
    ESP_LOGI(TAG, "Initializing wake word...");

    // 1. Load models
    char *partition_label = "model";
    models = esp_srmodel_init(partition_label);

    if (models == NULL)
    {
        ESP_LOGE(TAG, "Model partition missing or empty!");
        return false;
    }

    // 2. Configure AFE
    afe_handle = (esp_afe_sr_iface_t *)&ESP_AFE_SR_HANDLE;
    afe_config_t afe_config = AFE_CONFIG_DEFAULT(); // This should work now that headers are fixed

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

    // 3. Create AFE Instance
    esp_afe_sr_data_t *afe_data = afe_handle->create_from_config(&afe_config);
    if (afe_data == NULL)
    {
        ESP_LOGE(TAG, "Failed to create AFE instance");
        return false;
    }

    // 4. Start Tasks
    xTaskCreatePinnedToCore(&detect_task, "detect", 8 * 1024, (void *)afe_data, 5, NULL, 1);
    xTaskCreatePinnedToCore(&feed_task, "feed", 8 * 1024, (void *)afe_data, 5, NULL, 0);

    return true;
}

// ================= Main =================

void app_main(void)
{
    // Basic I2S Init for INMP441
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate = 16000, // Wake word usually requires 16k
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

    // Start Wake Word
    init_wake_word_detection();
}
