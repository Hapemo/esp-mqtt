#pragma once

#include <stdbool.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

// Default pins for ESP32 DevKit (can be changed in display_init params)
#define OLED_DEFAULT_SDA 21
#define OLED_DEFAULT_SCL 22
#define OLED_DEFAULT_ADDR 0x3C

typedef enum {
	DISPLAY_CTRL_SH1106 = 0,
	DISPLAY_CTRL_SSD1306 = 1,
} display_controller_t;

// Initialize the OLED display (defaults to SH1106 for backward compatibility)
// and start a small refresh task. Returns true on success.
bool display_init(int sda_gpio, int scl_gpio, int i2c_addr_7bit);

// Same API as display_init but lets you pick the controller variant explicitly.
bool display_init_with_controller(int sda_gpio,
								  int scl_gpio,
								  int i2c_addr_7bit,
								  display_controller_t controller);

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
