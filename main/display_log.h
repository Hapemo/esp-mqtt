#pragma once

#include <stdbool.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

// Default pins for ESP32 DevKit (can be changed in display_init params)
#define OLED_DEFAULT_SDA 4
#define OLED_DEFAULT_SCL 15
#define OLED_DEFAULT_ADDR 0x3C

// Initialize the OLED display and start a small refresh task.
// Returns true on success.
bool display_init(int sda_gpio, int scl_gpio, int i2c_addr_7bit);

// Print a line (truncated to fit). Adds to a 4-line rolling buffer and marks for redraw.
void display_println(const char *line);

// printf-style helper
void display_printf(const char *fmt, ...);

// Convenience helpers for your app events
void display_wifi_connected(const char *ip_str);
void display_wifi_disconnected(void);
void display_mqtt_connected(void);
void display_mqtt_disconnected(void);

#ifdef __cplusplus
}
#endif
