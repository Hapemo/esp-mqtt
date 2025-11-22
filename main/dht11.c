#include "sdkconfig.h"
#include "dht11.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

static const char *DHTTAG = "dht11";

// Busy-wait helper with timeout in microseconds; returns true if level == target before timeout
static bool wait_for_level(int gpio_num, int target_level, int timeout_us)
{
    int64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < timeout_us) {
        if (gpio_get_level((gpio_num_t)gpio_num) == target_level) return true;
    }
    return false;
}

esp_err_t dht11_read(int gpio_num, int *temperature_c, int *humidity_percent)
{
    if (!temperature_c || !humidity_percent) return ESP_ERR_INVALID_ARG;

    // 1) Start signal: drive low for >=18ms, then release
    gpio_config_t outcfg = {
        .pin_bit_mask = 1ULL << gpio_num,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&outcfg);
    gpio_set_level((gpio_num_t)gpio_num, 1);
    vTaskDelay(pdMS_TO_TICKS(1000)); // Let sensor stabilize
    gpio_set_level((gpio_num_t)gpio_num, 0);
    esp_rom_delay_us(20000); // >=18ms

    // Release line (input with pull-up)
    gpio_config_t incfg = outcfg;
    incfg.mode = GPIO_MODE_INPUT;
    incfg.pull_up_en = GPIO_PULLUP_ENABLE; // many modules already have a pull-up, this is a helper
    gpio_config(&incfg);

    // 2) Sensor response: LOW ~80us then HIGH ~80us
    if (!wait_for_level(gpio_num, 0, 100)) { 
        ESP_LOGE(DHTTAG, "Timeout waiting for LOW signal 1");
        return ESP_ERR_TIMEOUT; 
    }
    if (!wait_for_level(gpio_num, 1, 120)) { 
        ESP_LOGE(DHTTAG, "Timeout waiting for HIGH signal 2");
        return ESP_ERR_TIMEOUT; 
    }
    if (!wait_for_level(gpio_num, 0, 120)) { 
        ESP_LOGE(DHTTAG, "Timeout waiting for LOW signal 3");
        return ESP_ERR_TIMEOUT; 
    }

    // 3) Read 40 bits: for each bit, sensor keeps LOW (~50us) then HIGH: ~26us = 0, ~70us = 1
    uint8_t data[5] = {0};
    for (int bit = 0; bit < 40; ++bit) {
        // Wait for LOW-to-HIGH (end of low preamble)
    if (!wait_for_level(gpio_num, 1, 100)) { 
        ESP_LOGE(DHTTAG, "Timeout waiting for HIGH signal 4");
        return ESP_ERR_TIMEOUT;
    }
        int64_t t0 = esp_timer_get_time();
    // Measure length of HIGH pulse
    if (!wait_for_level(gpio_num, 0, 100)) { 
        ESP_LOGE(DHTTAG, "Timeout waiting for LOW signal 5");
        return ESP_ERR_TIMEOUT;
    }
        int64_t dt = esp_timer_get_time() - t0; // microseconds
        // Classify bit by HIGH duration threshold (~50us)
        int val = (dt > 50) ? 1 : 0;
        data[bit / 8] = (uint8_t)((data[bit / 8] << 1) | val);
    }

    // 4) Verify checksum: sum of first 4 bytes (8-bit) equals data[4]
    uint8_t sum = (uint8_t)(data[0] + data[1] + data[2] + data[3]);
    if (sum != data[4]) { return ESP_FAIL; }

    // DHT11 format: humidity = data[0], temperature = data[2] (integers, decimals are zero)
    *humidity_percent = (int)data[0];
    *temperature_c = (int)data[2];
    return ESP_OK;
}
