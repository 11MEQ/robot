// display_wrapper.h - SH1106 Display for ESP32-S3

#ifndef DISPLAY_WRAPPER_H
#define DISPLAY_WRAPPER_H

// ====================================================================================
// ESP32-S3 I2C Pin Configuration - CHANGE THESE TO MATCH YOUR WIRING
// ====================================================================================
#define CUSTOM_SDA 14   // GPIO 8 for SDA
#define CUSTOM_SCL 13  // GPIO 9 for SCL

// Common ESP32-S3 alternatives:
// SDA=21, SCL=22 (classic ESP32 pinout)
// SDA=41, SCL=42 (some boards)

// Screen dimensions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// ====================================================================================
// SH1106 Library Setup
// ====================================================================================
#include <U8g2lib.h>

// SH1106 128x64 OLED Display Constructor
// Using hardware I2C with custom pins
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ CUSTOM_SCL, /* data=*/ CUSTOM_SDA);

// Color constants
const int G_COLOR_WHITE = 1;
const int G_COLOR_BLACK = 0;

// ====================================================================================
// Generic Wrapper Functions
// ====================================================================================

inline void g_init_display() {
    display.begin();
    
    // Uncomment the line below if your display uses 0x3D instead of 0x3C
    // display.setI2CAddress(0x3D << 1);
    
    Serial.println(F("SH1106 Display initialized successfully!"));
}

inline void g_clear_display() {
    display.clearBuffer();
}

inline void g_update_display() {
    display.sendBuffer();
}

inline void g_draw_filled_round_rect(int x, int y, int w, int h, int r, int color) {
    display.setDrawColor(color);
    display.drawRBox(x, y, w, h, r);
}

inline void g_draw_filled_triangle(int x0, int y0, int x1, int y1, int x2, int y2, int color) {
    display.setDrawColor(color);
    display.drawTriangle(x0, y0, x1, y1, x2, y2);
}

#endif

/*
================================================================================
SETUP INSTRUCTIONS FOR SH1106 ON ESP32-S3
================================================================================

1. INSTALL LIBRARY:
   - Arduino IDE → Sketch → Include Library → Manage Libraries
   - Search: "U8g2"
   - Install: "U8g2 by oliver" (latest version)

2. WIRING:
   Display Pin → ESP32-S3 Pin
   VCC  → 3.3V (or 5V if supported)
   GND  → GND
   SDA  → GPIO 8
   SCL  → GPIO 9

3. ARDUINO IDE SETTINGS:
   Board: "ESP32S3 Dev Module"
   USB CDC On Boot: "Enabled" ← IMPORTANT FOR SERIAL!
   Upload Speed: 921600
   Flash Mode: QIO
   Flash Size: 4MB (or match your board)
   Partition Scheme: Default
   PSRAM: Disabled (unless needed)

4. PIN CONFIGURATION:
   If your board uses different I2C pins, edit lines 8-9 above:
   #define CUSTOM_SDA 8  ← Change to your SDA pin
   #define CUSTOM_SCL 9  ← Change to your SCL pin

5. I2C ADDRESS:
   Most SH1106 displays use 0x3C (default)
   If display doesn't work, try 0x3D:
   Uncomment line 35: display.setI2CAddress(0x3D << 1);

6. TROUBLESHOOTING:
   - Display blank? Check wiring and I2C address
   - Serial not working? Enable "USB CDC On Boot" in Tools menu
   - Wrong pins? Run I2C scanner (code below)

I2C SCANNER CODE (paste into Arduino IDE to find your display):
--------------------------------------------------------------------------------
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(8, 9); // SDA, SCL - adjust to your pins
  Serial.println("\nI2C Scanner Starting...");
}

void loop() {
  byte error, address;
  int devices = 0;
  
  Serial.println("Scanning I2C bus...");
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      devices++;
    }
  }
  
  if (devices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("Scan complete\n");
    
  delay(5000);
}
--------------------------------------------------------------------------------

================================================================================
*/