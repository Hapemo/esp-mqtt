#ifndef PMSX_MODULE_INCLUDE_PMSX_MODULE_H_
#define PMSX_MODULE_INCLUDE_PMSX_MODULE_H_

#include <stdbool.h>
#include <stdio.h>
#include "pmsx-data.h"

typedef void(*pm_data_callback_f)(pm_data_t*);

typedef struct {
    uint8_t sensor_id;
    uint8_t uart_port;
	bool indoor;
    bool enabled;
	bool periodic;
	int periodic_sec_interval;
	pm_data_callback_f callback;
	uint8_t set_pin;
	uint8_t reset_pin;
	uint8_t uart_tx_pin;
	uint8_t uart_rx_pin;
} pmsx003_config_t;

uint32_t idf_pmsx5003_init(pmsx003_config_t *config);

void idf_pmsx5003_destroy(pmsx003_config_t *config);

#endif