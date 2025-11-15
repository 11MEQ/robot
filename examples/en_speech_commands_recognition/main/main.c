/*
 * ESP32-S3-N16R8 Voice Assistant - FIXED VERSION
 * Fixes: SPIFFS mount, Model loading, Error handling
 * 
 * Hardware:
 * - INMP441: SD->GPIO10, WS->GPIO11, SCK->GPIO12
 * - SH1106 Display: SDA->GPIO14, SCL->GPIO13
 * - MAX98357A: DIN->GPIO17, BCLK->GPIO16, LRC->GPIO15
 * - RGB LED: GPIO48 (Built-in)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/i2s.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "esp_http_client.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_vfs_fat.h"

// Wake Word Detection (Optional)
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "model_path.h"

// Display
#include "u8g2.h"

static const char *TAG = "VOICE_ASSISTANT";

// ======================== Configuration ========================
#define WIFI_SSID      "AZ"
#define WIFI_PASS      "8/2*3-6/6=10HHa"
#define SERVER_URL     "http://192.168.1.3:5000/upload"

// I2S Configuration for INMP441 (Input)
#define I2S_NUM_MIC     (0)
#define SAMPLE_RATE     (16000)
#define I2S_BCK_MIC     (12)
#define I2S_WS_MIC      (11)
#define I2S_DI_MIC      (10)

// I2S Configuration for MAX98357A (Output)
#define I2S_NUM_SPK     (1)
#define I2S_DOUT_SPK    (17)
#define I2S_BCLK_SPK    (16)
#define I2S_LRC_SPK     (15)

// Display Configuration
#define DISPLAY_SDA     (14)
#define DISPLAY_SCL     (13)
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64

// RGB LED
#define RGB_LED_GPIO    (48)
#define RGB_BRIGHTNESS  100
#define RMT_TX_CHANNEL  RMT_CHANNEL_0

// Recording
#define SILENCE_THRESHOLD    500
#define SILENCE_DURATION_MS  1500
#define MAX_RECORDING_SEC    10
#define RECORDING_BUFFER_SIZE (SAMPLE_RATE * MAX_RECORDING_SEC * 2)

// Button for manual trigger (optional)
#define BUTTON_GPIO     (0)  // Boot button

// Playback buffer
#define PLAYBACK_BUFFER_SIZE (4096)

// WS2812 timing
#define WS2812_T0H_NS   (350)
#define WS2812_T0L_NS   (900)
#define WS2812_T1H_NS   (900)
#define WS2812_T1L_NS   (350)


// Add these globals at the top
#define STREAM_BUFFER_SIZE (8192)
// ======================== Animation Types ========================
typedef enum {
  WAKEUP = 0,
  RESET,
  MOVE_RIGHT_BIG,
  MOVE_LEFT_BIG,
  BLINK_LONG,
  BLINK_SHORT,
  HAPPY,
  SLEEP_ANIM,
  SACCADE_RANDOM,
  MAX_ANIMATIONS
} Animation;

// ======================== Eye State ========================
typedef struct {
  int height;
  int width;
  int x;
  int y;
} EyeState;

#define REF_EYE_HEIGHT 40
#define REF_EYE_WIDTH 40
#define REF_SPACE_BETWEEN_EYE 10
#define REF_CORNER_RADIUS 10

// ======================== Global Variables ========================
static esp_afe_sr_iface_t *afe_handle = NULL;
static volatile int task_flag = 0;
static srmodel_list_t *models = NULL;
static bool use_wake_word = false;  // Will be set based on model availability

static volatile bool pause_wake_word = false;
// Add this with other volatile flags
static volatile bool interrupt_playback = false;
static volatile bool pause_feed_task = false;  // ADD THIS LINE


static uint8_t *stream_buffer = NULL;
static volatile size_t buffer_write_pos = 0;
static volatile size_t buffer_read_pos = 0;
static volatile size_t buffer_data_size = 0;
static volatile bool stream_complete = false;
static SemaphoreHandle_t buffer_mutex = NULL;

static size_t get_buffered_data_size(void) {
    size_t size;
    xSemaphoreTake(buffer_mutex, portMAX_DELAY);
    size = buffer_data_size;
    xSemaphoreGive(buffer_mutex);
    return size;
}

static size_t stream_buffer_write(const uint8_t *data, size_t len) {
    xSemaphoreTake(buffer_mutex, portMAX_DELAY);
    
    size_t space_available = STREAM_BUFFER_SIZE - buffer_data_size;
    size_t to_write = (len < space_available) ? len : space_available;
    
    for (size_t i = 0; i < to_write; i++) {
        stream_buffer[buffer_write_pos] = data[i];
        buffer_write_pos = (buffer_write_pos + 1) % STREAM_BUFFER_SIZE;
    }
    
    buffer_data_size += to_write;
    xSemaphoreGive(buffer_mutex);
    
    return to_write;
}

static size_t stream_buffer_read(uint8_t *data, size_t len) {
    xSemaphoreTake(buffer_mutex, portMAX_DELAY);
    
    size_t to_read = (len < buffer_data_size) ? len : buffer_data_size;
    
    for (size_t i = 0; i < to_read; i++) {
        data[i] = stream_buffer[buffer_read_pos];
        buffer_read_pos = (buffer_read_pos + 1) % STREAM_BUFFER_SIZE;
    }
    
    buffer_data_size -= to_read;
    xSemaphoreGive(buffer_mutex);
    
    return to_read;
}

// Display
static u8g2_t u8g2;
static EyeState left_eye, right_eye;
static int corner_radius = REF_CORNER_RADIUS;

// Recording
static int16_t *recording_buffer = NULL;
static volatile bool is_recording = false;
static volatile int recording_samples = 0;
static volatile bool send_audio_flag = false;

// Playback
static FILE *playback_file = NULL;
static volatile bool audio_response_ready = false;
static volatile bool is_playing = false;

// Emotion
static char pending_emotion[64] = {0};
static volatile bool has_new_emotion = false;

// WiFi
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

// Button
static volatile bool button_pressed = false;

// ======================== RGB LED Functions ========================
static inline uint32_t ns_to_rmt_ticks(uint32_t ns) {
    return (ns * 80) / 1000;
}

void ws2812_write_led(uint8_t red, uint8_t green, uint8_t blue) {
    uint32_t color = (green << 16) | (red << 8) | blue;
    rmt_item32_t items[24];
    
    for (int bit = 0; bit < 24; bit++) {
        uint32_t bit_val = (color >> (23 - bit)) & 0x01;
        
        if (bit_val) {
            items[bit].level0 = 1;
            items[bit].duration0 = ns_to_rmt_ticks(WS2812_T1H_NS);
            items[bit].level1 = 0;
            items[bit].duration1 = ns_to_rmt_ticks(WS2812_T1L_NS);
        } else {
            items[bit].level0 = 1;
            items[bit].duration0 = ns_to_rmt_ticks(WS2812_T0H_NS);
            items[bit].level1 = 0;
            items[bit].duration1 = ns_to_rmt_ticks(WS2812_T0L_NS);
        }
    }
    
    rmt_write_items(RMT_TX_CHANNEL, items, 24, true);
    rmt_wait_tx_done(RMT_TX_CHANNEL, portMAX_DELAY);
}

void rgb_led_init(void) {
    rmt_config_t config = {
        .rmt_mode = RMT_MODE_TX,
        .channel = RMT_TX_CHANNEL,
        .gpio_num = RGB_LED_GPIO,
        .clk_div = 1,
        .mem_block_num = 1,
        .tx_config = {
            .carrier_en = false,
            .loop_en = false,
            .idle_level = RMT_IDLE_LEVEL_LOW,
            .idle_output_en = true,
        }
    };
    
    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
    ws2812_write_led(0, 0, 0);
}

void set_led_color(uint8_t r, uint8_t g, uint8_t b) {
    ws2812_write_led(r, g, b);
}

// ======================== Display Functions (Same as before) ========================
uint8_t u8g2_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch(msg) {
        case U8X8_MSG_GPIO_AND_DELAY_INIT:
            break;
        case U8X8_MSG_DELAY_MILLI:
            vTaskDelay(arg_int / portTICK_PERIOD_MS);
            break;
        case U8X8_MSG_DELAY_10MICRO:
            ets_delay_us(arg_int * 10);
            break;
        case U8X8_MSG_DELAY_100NANO:
            ets_delay_us(1);
            break;
    }
    return 1;
}

uint8_t u8g2_byte_i2c_hw_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    static uint8_t buffer[128];
    static uint8_t buf_idx = 0;
    uint8_t *data;
    
    switch(msg) {
        case U8X8_MSG_BYTE_SEND:
            data = (uint8_t *)arg_ptr;
            while(arg_int > 0) {
                buffer[buf_idx++] = *data;
                data++;
                arg_int--;
            }
            break;
        case U8X8_MSG_BYTE_INIT:
            break;
        case U8X8_MSG_BYTE_START_TRANSFER:
            buf_idx = 0;
            break;
        case U8X8_MSG_BYTE_END_TRANSFER:
        {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (0x3C << 1) | I2C_MASTER_WRITE, true);
            i2c_master_write(cmd, buffer, buf_idx, true);
            i2c_master_stop(cmd);
            i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);
            break;
        }
    }
    return 1;
}

void display_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = DISPLAY_SDA,
        .scl_io_num = DISPLAY_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
    
    u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8g2_byte_i2c_hw_cb, u8g2_gpio_and_delay_cb);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
    u8g2_ClearBuffer(&u8g2);
    u8g2_SendBuffer(&u8g2);
    
    ESP_LOGI(TAG, "✅ SH1106 Display initialized");
}

// ======================== Eye Animation Functions (Keep all from original) ========================
int calculate_safe_radius(int r, int w, int h) {
    if (w < 2 * (r + 1)) r = (w / 2) - 1;
    if (h < 2 * (r + 1)) r = (h / 2) - 1;
    return (r < 0) ? 0 : r;
}

void draw_eyes(void) {
    int r_left = calculate_safe_radius(corner_radius, left_eye.width, left_eye.height);
    int x_left = (int)(left_eye.x - left_eye.width / 2);
    int y_left = (int)(left_eye.y - left_eye.height / 2);
    u8g2_DrawRBox(&u8g2, x_left, y_left, left_eye.width, left_eye.height, r_left);

    int r_right = calculate_safe_radius(corner_radius, right_eye.width, right_eye.height);
    int x_right = (int)(right_eye.x - right_eye.width / 2);
    int y_right = (int)(right_eye.y - right_eye.height / 2);
    u8g2_DrawRBox(&u8g2, x_right, y_right, right_eye.width, right_eye.height, r_right);
}

void draw_frame(void) {
    u8g2_ClearBuffer(&u8g2);
    draw_eyes();
    u8g2_SendBuffer(&u8g2);
}

void reset_eyes(bool update) {
    left_eye.height = REF_EYE_HEIGHT;
    left_eye.width = REF_EYE_WIDTH;
    right_eye.height = REF_EYE_HEIGHT;
    right_eye.width = REF_EYE_WIDTH;
    
    left_eye.x = SCREEN_WIDTH / 2 - REF_EYE_WIDTH / 2 - REF_SPACE_BETWEEN_EYE / 2;
    left_eye.y = SCREEN_HEIGHT / 2;
    right_eye.x = SCREEN_WIDTH / 2 + REF_EYE_WIDTH / 2 + REF_SPACE_BETWEEN_EYE / 2;
    right_eye.y = SCREEN_HEIGHT / 2;

    corner_radius = REF_CORNER_RADIUS;
    
    if (update) draw_frame();
}

void sleep_eyes(void) {
    reset_eyes(false);
    left_eye.height = 2;
    left_eye.width = REF_EYE_WIDTH;
    right_eye.height = 2;
    right_eye.width = REF_EYE_WIDTH;
    corner_radius = 0;
    draw_frame();
}

void wakeup_animation(void) {
    reset_eyes(false);
    for(int h = 2; h <= REF_EYE_HEIGHT; h += 2) {
        left_eye.height = h;
        right_eye.height = h;    
        int mapped_radius = (h * REF_CORNER_RADIUS) / REF_EYE_HEIGHT;
        corner_radius = (mapped_radius < h/2) ? mapped_radius : h/2;
        draw_frame();
    }
}

// Keep all other animation functions from original...

// ======================== Emotion Processing ========================
void process_emotion(const char* emotion_code) {
    ESP_LOGI(TAG, "📥 Processing emotion: %s", emotion_code);
    
    char *underscore = strchr(emotion_code, '_');
    if (underscore) {
        char code[8] = {0};
        strncpy(code, emotion_code, underscore - emotion_code);
        
        if (strlen(code) >= 2) {
            char type = code[0];
            int num = atoi(&code[1]);
            
            if (type == 'A' || type == 'a') {
                if (num == 0) wakeup_animation();  // A0_Wakeup
                else reset_eyes(true);
            }
        }
    }
}

// ======================== WiFi Event Handler ========================
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Disconnected, retrying...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "✓ WiFi Connected! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        set_led_color(0, RGB_BRIGHTNESS, 0);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        set_led_color(0, 0, 0);
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// ======================== WiFi Initialization ========================
void wifi_init(void) {
    wifi_event_group = xEventGroupCreate();
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGI(TAG, "⚠️ Erasing NVS flash...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "✓ WiFi initialization started");
}

// ======================== SPIFFS Init (FIXED) ========================
esp_err_t spiffs_init(void) {
    ESP_LOGI(TAG, "Initializing SPIFFS...");
    
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "spiffs",  // Match partition name
        .max_files = 5,
        .format_if_mount_failed = true  // Auto-format if needed
    };
    
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ret;
    }
    
    size_t total = 0, used = 0;
    ret = esp_spiffs_info("spiffs", &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "✅ SPIFFS: Total: %d KB, Used: %d KB", total/1024, used/1024);
    }
    
    return ESP_OK;
}

// ======================== Audio Hardware Init ========================
esp_err_t inmp441_init(void) {
    ESP_LOGI(TAG, "Initializing INMP441...");
    
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2,
        .dma_buf_count = 6,
        .dma_buf_len = 640,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };
    
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK_MIC,
        .ws_io_num = I2S_WS_MIC,
        .data_out_num = -1,
        .data_in_num = I2S_DI_MIC
    };
    
    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_MIC, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_MIC, &pin_config));
    ESP_ERROR_CHECK(i2s_zero_dma_buffer(I2S_NUM_MIC));
    
    ESP_LOGI(TAG, "✓ INMP441 initialized");
    return ESP_OK;
}

esp_err_t max98357a_init(void) {
    ESP_LOGI(TAG, "Initializing MAX98357A...");
    
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = 22050,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 1024,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };
    
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCLK_SPK,
        .ws_io_num = I2S_LRC_SPK,
        .data_out_num = I2S_DOUT_SPK,
        .data_in_num = -1
    };
    
    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_SPK, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_SPK, &pin_config));
    ESP_ERROR_CHECK(i2s_zero_dma_buffer(I2S_NUM_SPK));
    
    ESP_LOGI(TAG, "✓ MAX98357A initialized");
    return ESP_OK;
}

// ======================== Audio Read ========================
int custom_get_feed_data(bool is_get_raw_channel, int16_t *buffer, int buffer_len) {
    size_t bytes_read = 0;
    int samples_count = buffer_len / sizeof(int16_t);
    int32_t *i2s_buffer = (int32_t *)malloc(samples_count * sizeof(int32_t));
    
    if (i2s_buffer == NULL) {
        return ESP_FAIL;
    }
    
    esp_err_t ret = i2s_read(I2S_NUM_MIC, i2s_buffer, samples_count * sizeof(int32_t), 
                             &bytes_read, portMAX_DELAY);
    
    if (ret != ESP_OK) {
        free(i2s_buffer);
        return ESP_FAIL;
    }
    
    int samples_read = bytes_read / sizeof(int32_t);
    for (int i = 0; i < samples_read; i++) {
        buffer[i] = (int16_t)(i2s_buffer[i] >> 14);
    }
    
    free(i2s_buffer);
    return samples_read * sizeof(int16_t);
}

// ======================== Calculate Audio Level ========================
int calculate_audio_level(int16_t *buffer, int samples) {
    long sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += abs(buffer[i]);
    }
    return sum / samples;
}

// ======================== HTTP Response Handler ========================
// Modified HTTP event handler for streaming
esp_err_t http_event_handler(esp_http_client_event_t *evt) {
    static char emotion_buffer[64] = {0};
    static bool started_playback = false;
    
    switch(evt->event_id) {
        case HTTP_EVENT_ON_HEADER:
            if (strcasecmp(evt->header_key, "X-Emotion-Code") == 0) {
                strncpy(emotion_buffer, evt->header_value, sizeof(emotion_buffer) - 1);
                strncpy(pending_emotion, emotion_buffer, sizeof(pending_emotion) - 1);
                has_new_emotion = true;
                ESP_LOGI(TAG, "📝 Emotion from header: %s", emotion_buffer);
            }
            break;
            
        case HTTP_EVENT_ON_DATA:
            if (evt->data_len > 0) {
                // Write data to circular buffer
                size_t written = 0;
                while (written < evt->data_len) {
                    size_t chunk = stream_buffer_write(
                        (uint8_t*)evt->data + written, 
                        evt->data_len - written
                    );
                    
                    if (chunk == 0) {
                        // Buffer full, wait a bit
                        vTaskDelay(10 / portTICK_PERIOD_MS);
                    } else {
                        written += chunk;
                    }
                }
                
                // Start playback once we have 2KB buffered
                if (!started_playback && get_buffered_data_size() >= 2048) {
                    started_playback = true;
                    audio_response_ready = true;
                    ESP_LOGI(TAG, "🎵 Starting streaming playback...");
                }
            }
            break;
            
        case HTTP_EVENT_ON_FINISH:
            stream_complete = true;
            if (!started_playback) {
                // Small file, start playback now
                audio_response_ready = true;
            }
            ESP_LOGI(TAG, "✅ Stream download complete");
            started_playback = false;
            memset(emotion_buffer, 0, sizeof(emotion_buffer));
            break;
            
        case HTTP_EVENT_ERROR:
        case HTTP_EVENT_DISCONNECTED:
            stream_complete = true;
            started_playback = false;
            memset(emotion_buffer, 0, sizeof(emotion_buffer));
            break;
            
        default:
            break;
    }
    return ESP_OK;
}

// New streaming playback function
void play_audio_stream(void) {
    ESP_LOGI(TAG, "🔊 Playing streaming audio...");
    is_playing = true;
    interrupt_playback = false;  // Reset interrupt flag
    
    #define PLAY_CHUNK_SIZE 1024
    int16_t *mono_buffer = (int16_t *)malloc(PLAY_CHUNK_SIZE);
    int16_t *stereo_buffer = (int16_t *)malloc(PLAY_CHUNK_SIZE * 2);
    
    if (!mono_buffer || !stereo_buffer) {
        ESP_LOGE(TAG, "Failed to allocate playback buffers");
        if (mono_buffer) free(mono_buffer);
        if (stereo_buffer) free(stereo_buffer);
        is_playing = false;
        return;
    }
    
    i2s_zero_dma_buffer(I2S_NUM_SPK);
    size_t bytes_written;
    
    // Play until stream is complete and buffer is empty, or interrupted
    while ((!stream_complete || get_buffered_data_size() > 0) && !interrupt_playback) {
        size_t bytes_read = stream_buffer_read((uint8_t*)mono_buffer, PLAY_CHUNK_SIZE);
        
        if (bytes_read > 0) {
            int samples = bytes_read / sizeof(int16_t);
            
            // Convert mono to stereo
            for (int i = 0; i < samples; i++) {
                stereo_buffer[i * 2] = mono_buffer[i];
                stereo_buffer[i * 2 + 1] = mono_buffer[i];
            }
            
            size_t stereo_bytes = samples * 2 * sizeof(int16_t);
            i2s_write(I2S_NUM_SPK, stereo_buffer, stereo_bytes, &bytes_written, portMAX_DELAY);
        } else {
            // Buffer empty, wait for more data
            if (!stream_complete && !interrupt_playback) {
                vTaskDelay(10 / portTICK_PERIOD_MS);
            } else {
                break;  // Stream done and buffer empty, or interrupted
            }
        }
        
        // Check for interruption more frequently
        if (interrupt_playback) {
            ESP_LOGI(TAG, "⏸️ Playback interrupted by wake word!");
            break;
        }
    }
    
    // Clear I2S buffer to stop audio immediately
    i2s_zero_dma_buffer(I2S_NUM_SPK);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    
    free(mono_buffer);
    free(stereo_buffer);
    is_playing = false;
    
    if (interrupt_playback) {
        ESP_LOGI(TAG, "✅ Playback stopped - ready for new command");
    } else {
        ESP_LOGI(TAG, "✅ Streaming playback finished");
    }
}

// ======================== Send Audio to Server ========================
void send_audio_to_server(void *arg) {
    ESP_LOGI(TAG, "📤 Sending %d samples to server...", recording_samples);
    set_led_color(0, 0, RGB_BRIGHTNESS);
    
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
    
    esp_http_client_config_t config = {
        .url = SERVER_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 60000,        // Increase to 60 seconds
        .buffer_size = 4096,        // Larger buffer for faster transfer
        .buffer_size_tx = 4096,     // Larger TX buffer
        .event_handler = http_event_handler,
    };
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/octet-stream");
    
    int audio_size = recording_samples * sizeof(int16_t);
    esp_http_client_set_post_field(client, (const char*)recording_buffer, audio_size);
    
    esp_err_t err = esp_http_client_perform(client);
    
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        int content_length = esp_http_client_get_content_length(client);
        ESP_LOGI(TAG, "✅ Audio sent! Status: %d, Response size: %d bytes", status, content_length);
        set_led_color(0, RGB_BRIGHTNESS, 0);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    } else {
        ESP_LOGE(TAG, "❌ Failed to send audio: %s", esp_err_to_name(err));
        
        // Show specific error info
        if (err == ESP_ERR_HTTP_FETCH_HEADER) {
            ESP_LOGE(TAG, "   → Server didn't respond with valid headers");
            ESP_LOGE(TAG, "   → Check if server is running and responsive");
        }
        
        set_led_color(RGB_BRIGHTNESS, 0, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
    set_led_color(0, 0, 0);
    esp_http_client_cleanup(client);
    send_audio_flag = false;
    
    vTaskDelete(NULL);
}

// ======================== Recording Task ========================
void recording_task(void *arg) {
    int chunk_size = 512;
    int16_t *chunk_buffer = (int16_t *)malloc(chunk_size * sizeof(int16_t));
    assert(chunk_buffer);
    
    int silence_counter = 0;
    int silence_threshold_chunks = (SILENCE_DURATION_MS * SAMPLE_RATE) / (chunk_size * 1000);
    
    ESP_LOGI(TAG, "🎙️ Recording started!");
    set_led_color(RGB_BRIGHTNESS, 0, 0);
    
    while (is_recording && recording_samples < (SAMPLE_RATE * MAX_RECORDING_SEC)) {
        int ret = custom_get_feed_data(false, chunk_buffer, chunk_size * sizeof(int16_t));
        
        if (ret > 0) {
            int samples_read = ret / sizeof(int16_t);
            int audio_level = calculate_audio_level(chunk_buffer, samples_read);
            
            if (recording_samples + samples_read < (SAMPLE_RATE * MAX_RECORDING_SEC)) {
                memcpy(&recording_buffer[recording_samples], chunk_buffer, samples_read * sizeof(int16_t));
                recording_samples += samples_read;
            }
            
            if (audio_level < SILENCE_THRESHOLD) {
                silence_counter++;
                if (silence_counter >= silence_threshold_chunks) {
                    ESP_LOGI(TAG, "🔇 Silence detected - Stopping recording");
                    break;
                }
            } else {
                silence_counter = 0;
            }
        }
        
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    
    is_recording = false;
    set_led_color(0, 0, 0);
    
    // Clear interrupt flag after recording is done
    interrupt_playback = false;
    
    ESP_LOGI(TAG, "⏹️ Recording stopped! Samples: %d (%.2f seconds)", 
             recording_samples, (float)recording_samples / SAMPLE_RATE);
    
    if (recording_samples > 0) {
        send_audio_flag = true;
        xTaskCreate(&send_audio_to_server, "send_audio", 8192, NULL, 5, NULL);
    }
    
    free(chunk_buffer);
    vTaskDelete(NULL);
}

// ======================== Button Handler ========================
void IRAM_ATTR button_isr_handler(void* arg) {
    button_pressed = true;
}

void button_init(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL);
    
    ESP_LOGI(TAG, "✓ Button initialized (GPIO%d)", BUTTON_GPIO);
}

// ======================== Button Monitor Task ========================
void button_monitor_task(void *arg) {
    ESP_LOGI(TAG, "✓ Button monitor started");
    
    while (task_flag) {
        if (button_pressed) {
            button_pressed = false;
            
            // Debounce
            vTaskDelay(50 / portTICK_PERIOD_MS);
            
            if (gpio_get_level(BUTTON_GPIO) == 0) {
                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "========================================");
                ESP_LOGI(TAG, "  ✅ BUTTON PRESSED - START RECORDING");
                ESP_LOGI(TAG, "========================================");
                
                wakeup_animation();
                
                // Wait for previous recording to finish
                while (send_audio_flag) {
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                }
                
                if (!is_recording) {
                    recording_samples = 0;
                    is_recording = true;
                    xTaskCreate(&recording_task, "recording", 4096, NULL, 5, NULL);
                }
                
                // Wait for button release
                while (gpio_get_level(BUTTON_GPIO) == 0) {
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                }
            }
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    vTaskDelete(NULL);
}

// 3. MODIFY feed_task - Add rate limiting to prevent overflow
void feed_task(void *arg) {
    esp_afe_sr_data_t *afe_data = (esp_afe_sr_data_t *)arg;
    int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
    int feed_channel = 1;
    int16_t *i2s_buff = (int16_t *)malloc(audio_chunksize * sizeof(int16_t) * feed_channel);
    assert(i2s_buff);
    
    int consecutive_errors = 0;
    const int MAX_CONSECUTIVE_ERRORS = 5;

    ESP_LOGI(TAG, "✓ Feed task started");

    while (task_flag) {
        // Pause feeding during recording, interrupt, OR explicit pause
        if (is_recording || interrupt_playback || pause_feed_task) {
            consecutive_errors = 0;  // Reset error count
            vTaskDelay(50 / portTICK_PERIOD_MS);
            continue;
        }
        
        int ret = custom_get_feed_data(false, i2s_buff, audio_chunksize * sizeof(int16_t) * feed_channel);
        
        if (ret > 0) {
            int feed_result = afe_handle->feed(afe_data, i2s_buff);
            
            if (feed_result == ESP_FAIL) {
                consecutive_errors++;
                
                if (consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
                    ESP_LOGW(TAG, "⚠️ Multiple AFE buffer overflows, flushing...");
                    flush_afe_buffer(afe_data);
                    consecutive_errors = 0;
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                } else {
                    vTaskDelay((10 * consecutive_errors) / portTICK_PERIOD_MS);
                }
            } else {
                consecutive_errors = 0;
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        } else {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
    
    if (i2s_buff) {
        free(i2s_buff);
    }
    vTaskDelete(NULL);
}


// 1. ADD BUFFER FLUSH FUNCTION (Add this after the feed_task function)
void flush_afe_buffer(esp_afe_sr_data_t *afe_data) {
    if (afe_handle && afe_data) {
        // Fetch and discard all pending data to clear the buffer
        for (int i = 0; i < 10; i++) {
            afe_fetch_result_t* res = afe_handle->fetch(afe_data);
            if (!res) break;
            vTaskDelay(5 / portTICK_PERIOD_MS);
        }
        ESP_LOGI(TAG, "✓ AFE buffer flushed");
    }
}



// 2. MODIFY detect_task - Add buffer flush after interrupt
void detect_task(void *arg) {
    esp_afe_sr_data_t *afe_data = (esp_afe_sr_data_t *)arg;
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  🎤 Wake Word Detection Active");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "قل كلمة التنبيه لبدء التسجيل");
    ESP_LOGI(TAG, "Say wake word to start recording");
    ESP_LOGI(TAG, "========================================");
    
    while (task_flag) {
        // Only pause detection during recording
        if (is_recording || interrupt_playback) {
            vTaskDelay(50 / portTICK_PERIOD_MS);
            continue;
        }
        
        afe_fetch_result_t* res = afe_handle->fetch(afe_data); 
        
        if (!res || res->ret_value == ESP_FAIL) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }

        if (res->wakeup_state == WAKENET_DETECTED) {
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "========================================");
            ESP_LOGI(TAG, "  ✅ WAKE WORD DETECTED!");
            ESP_LOGI(TAG, "========================================");
            
            // If currently playing, interrupt it
            if (is_playing) {
                ESP_LOGI(TAG, "⏸️ Interrupting current playback...");
                
                // CRITICAL: Stop feed task IMMEDIATELY
                pause_feed_task = true;
                interrupt_playback = true;
                
                // Quick visual feedback
                set_led_color(0, RGB_BRIGHTNESS, RGB_BRIGHTNESS);
                
                // Wait for playback to stop with timeout
                int timeout = 0;
                while (is_playing && timeout < 100) {
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                    timeout++;
                }
                
                // Flush AFE buffer
                ESP_LOGI(TAG, "🔄 Flushing AFE buffer...");
                flush_afe_buffer(afe_data);
                
                // CRITICAL: Keep feed paused for 500ms to let buffer fully clear
                ESP_LOGI(TAG, "⏸️ Keeping feed paused for stabilization...");
                vTaskDelay(500 / portTICK_PERIOD_MS);
                
                // Now safe to resume feeding
                pause_feed_task = false;
                ESP_LOGI(TAG, "▶️ Feed task resumed");
                
                set_led_color(0, 0, 0);
                
                // Quick eye blink
                left_eye.height = 2;
                right_eye.height = 2;
                draw_frame();
                vTaskDelay(50 / portTICK_PERIOD_MS);
                left_eye.height = REF_EYE_HEIGHT;
                right_eye.height = REF_EYE_HEIGHT;
                draw_frame();
                
            } else {
                // Full animation when not interrupting
                wakeup_animation();
            }
            
            // Wait for any pending operations
            while (send_audio_flag) {
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            
            if (!is_recording) {
                recording_samples = 0;
                is_recording = true;
                xTaskCreate(&recording_task, "recording", 4096, NULL, 5, NULL);
            }
            
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
    
    vTaskDelete(NULL);
}


// ======================== Simple MP3 Playback ========================
void play_mp3_file(const char *filename) {
    FILE *fp = fopen(filename, "rb");
    if (!fp) {
        ESP_LOGE(TAG, "Failed to open %s", filename);
        return;
    }
    
    ESP_LOGI(TAG, "🔊 Playing audio...");
    is_playing = true;
    
    // Allocate buffer for mono PCM input
    #define PLAY_CHUNK_SIZE 2048
    int16_t *mono_buffer = (int16_t *)malloc(PLAY_CHUNK_SIZE);
    // Allocate buffer for stereo output (double size)
    int16_t *stereo_buffer = (int16_t *)malloc(PLAY_CHUNK_SIZE * 2);
    
    if (!mono_buffer || !stereo_buffer) {
        ESP_LOGE(TAG, "Failed to allocate playback buffers");
        if (mono_buffer) free(mono_buffer);
        if (stereo_buffer) free(stereo_buffer);
        fclose(fp);
        return;
    }
    
    size_t bytes_read;
    size_t bytes_written;
    
    // Clear I2S DMA buffer before starting
    i2s_zero_dma_buffer(I2S_NUM_SPK);
    
    while ((bytes_read = fread(mono_buffer, 1, PLAY_CHUNK_SIZE, fp)) > 0) {
        int samples = bytes_read / sizeof(int16_t);
        
        // Convert mono to stereo by duplicating samples
        for (int i = 0; i < samples; i++) {
            stereo_buffer[i * 2] = mono_buffer[i];     // Left channel
            stereo_buffer[i * 2 + 1] = mono_buffer[i]; // Right channel
        }
        
        // Write stereo data to I2S
        size_t stereo_bytes = samples * 2 * sizeof(int16_t);
        i2s_write(I2S_NUM_SPK, stereo_buffer, stereo_bytes, &bytes_written, portMAX_DELAY);
        
        // Small delay to prevent buffer overrun
        vTaskDelay(1);
    }
    
    // Wait for DMA to finish
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    free(mono_buffer);
    free(stereo_buffer);
    fclose(fp);
    is_playing = false;
    
    ESP_LOGI(TAG, "✅ Playback finished");
}

// ======================== Audio Playback Task ========================
// Modified audio playback task
void audio_playback_task(void *arg) {
    ESP_LOGI(TAG, "✓ Audio playback task started");
    
    while (task_flag) {
        if (audio_response_ready) {
            audio_response_ready = false;
            
            ESP_LOGI(TAG, "\n╔══════════════════════════════════════════╗");
            ESP_LOGI(TAG, "║      🔊 PLAYING RESPONSE                ║");
            ESP_LOGI(TAG, "╚══════════════════════════════════════════╝");
            
            // Process emotion
            if (has_new_emotion) {
                process_emotion(pending_emotion);
                has_new_emotion = false;
                memset(pending_emotion, 0, sizeof(pending_emotion));
            }
            
            // Play streaming audio (can be interrupted)
            play_audio_stream();
            
            // Clean up stream state
            stream_complete = false;
            buffer_write_pos = 0;
            buffer_read_pos = 0;
            buffer_data_size = 0;
            interrupt_playback = false;
            
            // Short delay before ready for next command
            vTaskDelay(200 / portTICK_PERIOD_MS);
            
            // Return to idle animation only if not interrupted
            if (!is_recording) {
                reset_eyes(true);
            }
        }
        
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    vTaskDelete(NULL);
}


// ======================== Wake Word Initialization ========================
// 4. ALTERNATIVE: Increase AFE buffer size in init_wake_word_detection()
bool init_wake_word_detection(void) {
    ESP_LOGI(TAG, "Attempting to initialize wake word detection...");
    
    // Load wake word models from partition
    char *partition_label = "model";
    models = esp_srmodel_init(partition_label);
    
    if (models == NULL) {
        ESP_LOGW(TAG, "⚠️ Wake word models not found");
        ESP_LOGW(TAG, "→ Falling back to button-triggered recording");
        return false;
    }
    
    ESP_LOGI(TAG, "✓ Speech models loaded successfully");
    
    // Initialize AFE
    afe_handle = (esp_afe_sr_iface_t *)&ESP_AFE_SR_HANDLE;
    afe_config_t afe_config = AFE_CONFIG_DEFAULT();
    afe_config.wakenet_model_name = esp_srmodel_filter(models, ESP_WN_PREFIX, NULL);
    
    if (afe_config.wakenet_model_name == NULL) {
        ESP_LOGW(TAG, "⚠️ No wake-net model found");
        return false;
    }
    
    // === KEY FIX: Increase buffer sizes ===
    afe_config.aec_init = false;
    afe_config.pcm_config.total_ch_num = 1;
    afe_config.pcm_config.mic_num = 1;
    afe_config.pcm_config.ref_num = 0;
    
    // Increase internal buffer sizes to prevent overflow
    afe_config.afe_ringbuf_size = 100;  // Increase from default (usually 50)
    
    ESP_LOGI(TAG, "Wake word model: %s", afe_config.wakenet_model_name);
    
    esp_afe_sr_data_t *afe_data = afe_handle->create_from_config(&afe_config);
    if (afe_data == NULL) {
        ESP_LOGW(TAG, "⚠️ Failed to create AFE instance");
        return false;
    }
    
    // Create tasks with proper priority
    // Lower priority for feed to prevent overwhelming the buffer
    xTaskCreatePinnedToCore(&detect_task, "detect", 8 * 1024, (void*)afe_data, 5, NULL, 1);
    xTaskCreatePinnedToCore(&feed_task, "feed", 8 * 1024, (void*)afe_data, 4, NULL, 0);  // Lower priority
    
    ESP_LOGI(TAG, "✅ Wake word detection initialized!");
    return true;
}

// ======================== Main Function ========================
void app_main(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Voice Assistant - ESP32-S3-N16R8");
    ESP_LOGI(TAG, "  STREAMING VERSION");
    ESP_LOGI(TAG, "========================================");
    
    // Initialize RGB LED
    rgb_led_init();
    ESP_LOGI(TAG, "✅ RGB LED initialized");
    
    // Initialize Display
    display_init();
    sleep_eyes();
    
    // Initialize SPIFFS (for model storage, not audio)
    esp_err_t spiffs_ret = spiffs_init();
    if (spiffs_ret != ESP_OK) {
        ESP_LOGE(TAG, "⚠️ SPIFFS initialization failed");
    }
    
    // Allocate recording buffer in PSRAM
    recording_buffer = (int16_t *)heap_caps_malloc(RECORDING_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    if (recording_buffer == NULL) {
        ESP_LOGE(TAG, "❌ Failed to allocate recording buffer!");
        return;
    }
    ESP_LOGI(TAG, "✓ Recording buffer allocated: %d bytes", RECORDING_BUFFER_SIZE);
    
    // Initialize streaming buffer in PSRAM for better performance
    stream_buffer = (uint8_t *)heap_caps_malloc(STREAM_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    if (stream_buffer == NULL) {
        ESP_LOGE(TAG, "❌ Failed to allocate stream buffer!");
        free(recording_buffer);
        return;
    }
    ESP_LOGI(TAG, "✓ Stream buffer allocated: %d bytes", STREAM_BUFFER_SIZE);
    
    // Create buffer mutex for thread-safe access
    buffer_mutex = xSemaphoreCreateMutex();
    if (buffer_mutex == NULL) {
        ESP_LOGE(TAG, "❌ Failed to create buffer mutex!");
        free(recording_buffer);
        free(stream_buffer);
        return;
    }
    ESP_LOGI(TAG, "✓ Buffer mutex created");
    
    // Initialize stream state
    buffer_write_pos = 0;
    buffer_read_pos = 0;
    buffer_data_size = 0;
    stream_complete = false;
    
    // Initialize WiFi
    wifi_init();
    ESP_LOGI(TAG, "⏳ Waiting for WiFi connection...");
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, 
                        pdMS_TO_TICKS(10000));
    
    // Initialize Audio Hardware
    inmp441_init();
    max98357a_init();
    
    // Initialize Button
    button_init();
    
    // Try to initialize wake word detection
    task_flag = 1;
    use_wake_word = init_wake_word_detection();
    
    if (!use_wake_word) {
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "╔════════════════════════════════════════╗");
        ESP_LOGI(TAG, "║  MODE: BUTTON-TRIGGERED RECORDING     ║");
        ESP_LOGI(TAG, "╚════════════════════════════════════════╝");
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "📌 Press BOOT button (GPIO0) to record");
        ESP_LOGI(TAG, "");
        
        // Start button monitor task instead of wake word
        xTaskCreatePinnedToCore(&button_monitor_task, "button", 4 * 1024, NULL, 5, NULL, 1);
    }
    
    // Start audio playback task (handles streaming playback)
    xTaskCreatePinnedToCore(&audio_playback_task, "playback", 8 * 1024, NULL, 4, NULL, 1);
    
    ESP_LOGI(TAG, "✓ System ready!");
    ESP_LOGI(TAG, "🎵 Streaming mode enabled - instant playback!");
    
    // Wake up animation
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    wakeup_animation();
    
    // Blink LED to show ready state
    for (int i = 0; i < 3; i++) {
        set_led_color(0, RGB_BRIGHTNESS, 0);
        vTaskDelay(200 / portTICK_PERIOD_MS);
        set_led_color(0, 0, 0);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔══════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  ✅ READY FOR VOICE COMMANDS            ║");
    ESP_LOGI(TAG, "║  🎯 Streaming: ~0.5s response latency   ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    
    // Main loop
    while(1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}