set(srcs
    "main.c"
    )

set(requires
    esp-sr              # Wake Word Detection
    led_strip           # RGB LED control
    hardware_driver     # Hardware definitions
    nvs_flash           # WiFi settings storage
    esp_wifi            # WiFi
    esp_netif           # Network Interface
    esp_event           # Event Loop
    esp_http_client     # HTTP Client
    driver              # I2S, I2C, GPIO, RMT
    esp_system          # System functions
    freertos            # FreeRTOS
    spiffs              # File system
    u8g2-master         # Display library
    )

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "."
                       REQUIRES ${requires})

# Disable warnings for cleaner output
component_compile_options(-w)