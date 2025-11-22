#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Read temperature (C) and humidity (%) from a DHT11 sensor on the given GPIO.
// Returns ESP_OK on success, else ESP_ERR_TIMEOUT / ESP_FAIL on bad checksum or timing.
esp_err_t dht11_read(int gpio_num, int *temperature_c, int *humidity_percent);

#ifdef __cplusplus
}
#endif
