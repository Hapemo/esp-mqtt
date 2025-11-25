#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "display_log.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
// #include "esp_lcd_panel_ssd1306.h" // Not used in SH1106 path

static const char *DLTAG = "display";

static bool s_display_ok = false;
static int s_addr_7bit = 0x3C;
static esp_lcd_panel_io_handle_t s_io = NULL;
static esp_lcd_panel_handle_t s_panel = NULL; // unused for SH1106 path
static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_dev = NULL;
static display_controller_t s_controller = DISPLAY_CTRL_SH1106;

#define MAX_LINES 4
#define MAX_CHARS 21
static char s_lines[MAX_LINES][MAX_CHARS + 1]; // s_lines holds the lines to be printed.
static int s_head = 0; // next line index to write
static SemaphoreHandle_t s_mutex;
static volatile bool s_dirty = false;

// 6x8 font, ASCII 32..127. Each char is 6 columns, least significant bit is top pixel.
static const uint8_t font6x8[96][6] = {
    {0,0,0,0,0,0}, // 32 ' '
    {0x00,0x00,0x5F,0x00,0x00,0x00}, // !
    {0x00,0x03,0x00,0x03,0x00,0x00}, // "
    {0x14,0x7F,0x14,0x7F,0x14,0x00}, // #
    {0x24,0x2A,0x7F,0x2A,0x12,0x00}, // $
    {0x23,0x13,0x08,0x64,0x62,0x00}, // %
    {0x36,0x49,0x55,0x22,0x50,0x00}, // &
    {0x00,0x05,0x03,0x00,0x00,0x00}, // '
    {0x00,0x1C,0x22,0x41,0x00,0x00}, // (
    {0x00,0x41,0x22,0x1C,0x00,0x00}, // )
    {0x14,0x08,0x3E,0x08,0x14,0x00}, // *
    {0x08,0x08,0x3E,0x08,0x08,0x00}, // +
    {0x00,0x50,0x30,0x00,0x00,0x00}, // ,
    {0x08,0x08,0x08,0x08,0x08,0x00}, // -
    {0x00,0x60,0x60,0x00,0x00,0x00}, // .
    {0x20,0x10,0x08,0x04,0x02,0x00}, // /
    {0x3E,0x51,0x49,0x45,0x3E,0x00}, // 0
    {0x00,0x42,0x7F,0x40,0x00,0x00}, // 1
    {0x42,0x61,0x51,0x49,0x46,0x00}, // 2
    {0x21,0x41,0x45,0x4B,0x31,0x00}, // 3
    {0x18,0x14,0x12,0x7F,0x10,0x00}, // 4
    {0x27,0x45,0x45,0x45,0x39,0x00}, // 5
    {0x3C,0x4A,0x49,0x49,0x30,0x00}, // 6
    {0x01,0x71,0x09,0x05,0x03,0x00}, // 7
    {0x36,0x49,0x49,0x49,0x36,0x00}, // 8
    {0x06,0x49,0x49,0x29,0x1E,0x00}, // 9
    {0x00,0x36,0x36,0x00,0x00,0x00}, // :
    {0x00,0x56,0x36,0x00,0x00,0x00}, // ;
    {0x08,0x14,0x22,0x41,0x00,0x00}, // <
    {0x14,0x14,0x14,0x14,0x14,0x00}, // =
    {0x00,0x41,0x22,0x14,0x08,0x00}, // >
    {0x02,0x01,0x51,0x09,0x06,0x00}, // ?
    {0x32,0x49,0x79,0x41,0x3E,0x00}, // @
    {0x7E,0x11,0x11,0x11,0x7E,0x00}, // A
    {0x7F,0x49,0x49,0x49,0x36,0x00}, // B
    {0x3E,0x41,0x41,0x41,0x22,0x00}, // C
    {0x7F,0x41,0x41,0x22,0x1C,0x00}, // D
    {0x7F,0x49,0x49,0x49,0x41,0x00}, // E
    {0x7F,0x09,0x09,0x09,0x01,0x00}, // F
    {0x3E,0x41,0x49,0x49,0x7A,0x00}, // G
    {0x7F,0x08,0x08,0x08,0x7F,0x00}, // H
    {0x00,0x41,0x7F,0x41,0x00,0x00}, // I
    {0x20,0x40,0x41,0x3F,0x01,0x00}, // J
    {0x7F,0x08,0x14,0x22,0x41,0x00}, // K
    {0x7F,0x40,0x40,0x40,0x40,0x00}, // L
    {0x7F,0x02,0x0C,0x02,0x7F,0x00}, // M
    {0x7F,0x04,0x08,0x10,0x7F,0x00}, // N
    {0x3E,0x41,0x41,0x41,0x3E,0x00}, // O
    {0x7F,0x09,0x09,0x09,0x06,0x00}, // P
    {0x3E,0x41,0x51,0x21,0x5E,0x00}, // Q
    {0x7F,0x09,0x19,0x29,0x46,0x00}, // R
    {0x46,0x49,0x49,0x49,0x31,0x00}, // S
    {0x01,0x01,0x7F,0x01,0x01,0x00}, // T
    {0x3F,0x40,0x40,0x40,0x3F,0x00}, // U
    {0x1F,0x20,0x40,0x20,0x1F,0x00}, // V
    {0x3F,0x40,0x38,0x40,0x3F,0x00}, // W
    {0x63,0x14,0x08,0x14,0x63,0x00}, // X
    {0x07,0x08,0x70,0x08,0x07,0x00}, // Y
    {0x61,0x51,0x49,0x45,0x43,0x00}, // Z
    {0x00,0x7F,0x41,0x41,0x00,0x00}, // [
    {0x02,0x04,0x08,0x10,0x20,0x00}, // 
    {0x00,0x41,0x41,0x7F,0x00,0x00}, // ]
    {0x04,0x02,0x01,0x02,0x04,0x00}, // ^
    {0x40,0x40,0x40,0x40,0x40,0x00}, // _
    {0x00,0x01,0x02,0x00,0x00,0x00}, // `
    {0x20,0x54,0x54,0x54,0x78,0x00}, // a
    {0x7F,0x48,0x44,0x44,0x38,0x00}, // b
    {0x38,0x44,0x44,0x44,0x20,0x00}, // c
    {0x38,0x44,0x44,0x48,0x7F,0x00}, // d
    {0x38,0x54,0x54,0x54,0x18,0x00}, // e
    {0x08,0x7E,0x09,0x01,0x02,0x00}, // f
    {0x0C,0x52,0x52,0x52,0x3E,0x00}, // g
    {0x7F,0x08,0x04,0x04,0x78,0x00}, // h
    {0x00,0x44,0x7D,0x40,0x00,0x00}, // i
    {0x20,0x40,0x44,0x3D,0x00,0x00}, // j
    {0x7F,0x10,0x28,0x44,0x00,0x00}, // k
    {0x00,0x41,0x7F,0x40,0x00,0x00}, // l
    {0x7C,0x04,0x18,0x04,0x78,0x00}, // m
    {0x7C,0x08,0x04,0x04,0x78,0x00}, // n
    {0x38,0x44,0x44,0x44,0x38,0x00}, // o
    {0x7C,0x14,0x14,0x14,0x08,0x00}, // p
    {0x08,0x14,0x14,0x18,0x7C,0x00}, // q
    {0x7C,0x08,0x04,0x04,0x08,0x00}, // r
    {0x48,0x54,0x54,0x54,0x24,0x00}, // s
    {0x04,0x3F,0x44,0x40,0x20,0x00}, // t
    {0x3C,0x40,0x40,0x20,0x7C,0x00}, // u
    {0x1C,0x20,0x40,0x20,0x1C,0x00}, // v
    {0x3C,0x40,0x30,0x40,0x3C,0x00}, // w
    {0x44,0x28,0x10,0x28,0x44,0x00}, // x
    {0x0C,0x50,0x50,0x50,0x3C,0x00}, // y
    {0x44,0x64,0x54,0x4C,0x44,0x00}, // z
    {0x08,0x36,0x41,0x00,0x00,0x00}, // {
    {0x00,0x00,0x7F,0x00,0x00,0x00}, // |
    {0x00,0x41,0x36,0x08,0x00,0x00}, // }
    {0x10,0x08,0x10,0x08,0x00,0x00}, // ~
    {0,0,0,0,0,0}, // 127
};

static inline void fb_putpix(uint8_t *fb, int x, int y) {
    if ((unsigned)x >= 128 || (unsigned)y >= 64) return;
    int index = (y / 8) * 128 + x;
    fb[index] |= (1u << (y & 7));
}

// Draw character c at (x,y) in framebuffer fb
static void fb_draw_char(uint8_t *fb, int x, int y, char c) {
    if (c < 32 || c > 127) c = '?';
    const uint8_t *cols = font6x8[c - 32];
    for (int dx = 0; dx < 6; ++dx) {
        uint8_t bits = cols[dx];
        for (int dy = 0; dy < 8; ++dy) {
            if (bits & (1u << dy)) fb_putpix(fb, x + dx, y + dy);
        }
    }
}

// Draw string s at (x,y) in framebuffer fb, using fb_draw_char
static void fb_draw_str(uint8_t *fb, int x, int y, const char *s) {
    int cx = x;
    while (*s) {
        fb_draw_char(fb, cx, y, *s++);
        cx += 6;
        if (cx > 122) break;
    }
}

// ---------------- OLED low-level helpers ----------------
// SH1106 has 132x64 GDDRAM, visible 128 columns start at +2 offset.
#define SH1106_COL_OFFSET 0      // Common modules need +2. If you see left-edge noise, try 0 or 2.
#define SH1106_COL_TOTAL 132     // SH1106 GDDRAM columns

static inline void oled_cmd(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd}; // control=0x00 (command) marks next byte as command
    (void)i2c_master_transmit(s_dev, buf, sizeof(buf), 50);
}

// Set OLED page and column address (page = 0..7, column = 0..127)
static inline void oled_set_page_col(int page, int col) {
    oled_cmd(0xB0 | (page & 0x0F));
    int col_offset = (s_controller == DISPLAY_CTRL_SH1106) ? SH1106_COL_OFFSET : 0;
    int c = col + col_offset;
    oled_cmd(0x10 | ((c >> 4) & 0x0F));    // higher col bits
    oled_cmd(0x00 | (c & 0x0F));           // lower col bits
}

// Transmit data bytes to OLED
static inline void oled_tx_data(const void *data, size_t len) {
    // Chunk data with 0x40 control prefix (data)
    const uint8_t *p = (const uint8_t *)data;
    const size_t chunk = 16; // conservative chunk size
    uint8_t buf[1 + chunk];
    buf[0] = 0x40;
    while (len) {
        size_t n = len > chunk ? chunk : len;
        memcpy(&buf[1], p, n);
        (void)i2c_master_transmit(s_dev, buf, 1 + n, 100);
        p += n;
        len -= n;
    }
}

// Draw full 128x64 framebuffer to SH1106 display
static void sh1106_draw_full(const uint8_t *fb) {
    static const uint8_t zeros8[8] = {0};
    for (int page = 0; page < 8; ++page) {
        oled_set_page_col(page, 0);

        int left = SH1106_COL_OFFSET;
        while (left > 0) {
            size_t n = left > (int)sizeof(zeros8) ? sizeof(zeros8) : (size_t)left;
            oled_tx_data(zeros8, n);
            left -= (int)n;
        }

        const uint8_t *row = &fb[page * 128];
        oled_tx_data(row, 128);

        int right = SH1106_COL_TOTAL - (SH1106_COL_OFFSET + 128);
        while (right > 0) {
            size_t n = right > (int)sizeof(zeros8) ? sizeof(zeros8) : (size_t)right;
            oled_tx_data(zeros8, n);
            right -= (int)n;
        }
    }
}

static void ssd1306_draw_full(const uint8_t *fb) {
    for (int page = 0; page < 8; ++page) {
        oled_set_page_col(page, 0);
        const uint8_t *row = &fb[page * 128];
        oled_tx_data(row, 128);
    }
}

static void oled_draw_full(const uint8_t *fb) {
    if (s_controller == DISPLAY_CTRL_SH1106) {
        sh1106_draw_full(fb);
    } else {
        ssd1306_draw_full(fb);
    }
}

static void sh1106_init_sequence(void) {
    oled_cmd(0xAE);                 // display OFF
    oled_cmd(0xD5); oled_cmd(0x80); // clock divide
    oled_cmd(0xA8); oled_cmd(0x3F); // multiplex 1/64
    oled_cmd(0xD3); oled_cmd(0x00); // display offset
    oled_cmd(0x40 | 0x00);          // start line 0
    oled_cmd(0xA1);                 // segment remap (mirror X for common modules)
    oled_cmd(0xC8);                 // COM scan dec (mirror Y)
    oled_cmd(0xDA); oled_cmd(0x12); // COM pins config
    oled_cmd(0x81); oled_cmd(0x7F); // contrast
    oled_cmd(0xD9); oled_cmd(0x22); // pre-charge
    oled_cmd(0xDB); oled_cmd(0x35); // VCOMH
    oled_cmd(0xA4);                 // resume to RAM
    oled_cmd(0xA6);                 // normal display
    oled_cmd(0xAF);                 // display ON
}

static void ssd1306_init_sequence(void) {
    oled_cmd(0xAE);                 // display OFF
    oled_cmd(0xD5); oled_cmd(0x80); // display clock
    oled_cmd(0xA8); oled_cmd(0x3F); // multiplex ratio 1/64
    oled_cmd(0xD3); oled_cmd(0x00); // display offset
    oled_cmd(0x40 | 0x00);          // start line 0
    oled_cmd(0x8D); oled_cmd(0x14); // charge pump enable
    oled_cmd(0x20); oled_cmd(0x00); // horizontal addressing
    oled_cmd(0xA1);                 // segment remap
    oled_cmd(0xC8);                 // COM scan dec
    oled_cmd(0xDA); oled_cmd(0x12); // COM pins config for 128x64
    oled_cmd(0x81); oled_cmd(0x7F); // contrast
    oled_cmd(0xD9); oled_cmd(0xF1); // pre-charge period
    oled_cmd(0xDB); oled_cmd(0x40); // VCOMH deselect
    oled_cmd(0xA4);                 // resume to RAM
    oled_cmd(0xA6);                 // normal (not inverted)
    oled_cmd(0xAF);                 // display ON
}

// Draw all the lines from s_lines[] to the display
static void draw_all(void) {
    if (!s_display_ok) return;
    static uint8_t fb[128 * 64 / 8]; // framebuffer
    memset(fb, 0x00, sizeof(fb));
    for (int i = 0; i < MAX_LINES; ++i) {
        int idx = (s_head + i) % MAX_LINES;
        int y = 16 * i; // line spacing
        if (s_lines[idx][0] != '\0') fb_draw_str(fb, 0, y, s_lines[idx]);
    }
    oled_draw_full(fb);
}

static void display_task(void *arg) {
    (void)arg;
    for (;;) {
        if (s_dirty) {
            if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                draw_all();
                s_dirty = false;
                xSemaphoreGive(s_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

static void push_line(const char *text) {
    if (!s_display_ok) return;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
    // Truncate and copy
    size_t n = strlen(text);
    if (n > MAX_CHARS) n = MAX_CHARS;
    memset(s_lines[s_head], 0, MAX_CHARS + 1);
    memcpy(s_lines[s_head], text, n);
    s_head = (s_head + 1) % MAX_LINES;
    s_dirty = true;
    xSemaphoreGive(s_mutex);
}

bool display_init_with_controller(int sda_gpio,
                                  int scl_gpio,
                                  int i2c_addr_7bit,
                                  display_controller_t controller) {
    s_controller = controller;
    // Init ring buffer
    for (int i = 0; i < MAX_LINES; ++i) {
        s_lines[i][0] = '\0';
    }
    s_head = 0;

    if (i2c_addr_7bit > 0)
        s_addr_7bit = i2c_addr_7bit;

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        ESP_LOGE(DLTAG, "Failed to create display mutex");
        return false;
    }

    // Initialize I2C master bus (IDF v5.1+ API)
    int sda = sda_gpio <= 0 ? OLED_DEFAULT_SDA : sda_gpio;
    int scl = scl_gpio <= 0 ? OLED_DEFAULT_SCL : scl_gpio;
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };
    if (i2c_new_master_bus(&bus_cfg, &s_bus) != ESP_OK) {
        ESP_LOGE(DLTAG, "Failed to create I2C master bus");
        return false;
    }

    // Detect I2C address (commonly 0x3C or 0x3D)
    uint8_t try_addr = (i2c_addr_7bit > 0) ? (uint8_t)i2c_addr_7bit : (uint8_t)s_addr_7bit;
    esp_err_t probe_res = i2c_master_probe(s_bus, try_addr, 100);
    if (probe_res != ESP_OK) {
        uint8_t alt = (try_addr == 0x3C) ? 0x3D : 0x3C;
        ESP_LOGW(DLTAG, "I2C addr 0x%02X not responding, trying 0x%02X", try_addr, alt);
        if (i2c_master_probe(s_bus, alt, 100) == ESP_OK) {
            try_addr = alt;
        } else {
            // Quick scan 0x3C..0x3F for debugging
            for (uint8_t a = 0x3C; a <= 0x3F; ++a) {
                if (i2c_master_probe(s_bus, a, 50) == ESP_OK) {
                    ESP_LOGI(DLTAG, "Found device at 0x%02X", a);
                }
            }
            ESP_LOGE(DLTAG, "No OLED controller responding on I2C (check wiring SDA=%d SCL=%d, power, GND)", sda, scl);
            return false;
        }
    }
    s_addr_7bit = try_addr;
    ESP_LOGI(DLTAG, "Using I2C addr 0x%02X for OLED", s_addr_7bit);

    // Create raw I2C device for SH1106 writes
    i2c_device_config_t dev_cfg = {
        .device_address = (uint8_t)s_addr_7bit,
        .scl_speed_hz = 50000, // slow for robustness
    };
    if (i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev) != ESP_OK) {
        ESP_LOGE(DLTAG, "i2c_master_bus_add_device failed");
        return false;
    }

    if (s_controller == DISPLAY_CTRL_SH1106) {
        sh1106_init_sequence();
    } else {
        ssd1306_init_sequence();
    }

    // White splash by writing GDDRAM
    {
        static uint8_t fbw[128 * 64 / 8];
        memset(fbw, 0xFF, sizeof(fbw));
        oled_draw_full(fbw);
    }
    // Keep the white screen visible a bit before first text
    vTaskDelay(pdMS_TO_TICKS(300));

    s_display_ok = true;

    // Initial screen (after white fill)
    strncpy(s_lines[0], "Booting...", MAX_CHARS);
    s_head = 1;
    draw_all();
    ESP_LOGI(DLTAG, "%s init done, drew initial screen",
             (s_controller == DISPLAY_CTRL_SH1106) ? "SH1106" : "SSD1306");

    // Start updater task
    // This will run a background task to update the display when new lines are added.
    // Via the flag s_dirty.
    xTaskCreate(display_task, "display_task", 2048, NULL, 4, NULL);

    return true;
}

bool display_init(int sda_gpio, int scl_gpio, int i2c_addr_7bit) {
    return display_init_with_controller(sda_gpio, scl_gpio, i2c_addr_7bit, DISPLAY_CTRL_SSD1306);
}

void display_println(const char *line) {
    if (!line) return;
    push_line(line);
}

void display_printf(const char *fmt, ...) {
    char buf[64];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    push_line(buf);
}

void display_wifi_connected(const char *ip_str) {
    display_println("WiFi: OK");
    if (ip_str) display_printf("IP %s", ip_str);
}

void display_wifi_disconnected(void) {
    display_println("WiFi: DISC");
}

void display_mqtt_connected(void) {
    display_println("MQTT: OK");
}

void display_mqtt_disconnected(void) {
    display_println("MQTT: DISC");
}
