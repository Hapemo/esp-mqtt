#ifndef PMSX_DATA_INCLUDE_PMSX_DATA_H_
#define PMSX_DATA_INCLUDE_PMSX_DATA_H_

#include <stdio.h>

typedef struct {
	uint16_t pm1_0;
	uint16_t pm2_5;
	uint16_t pm10;
	uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
	uint8_t sensor_id;
} pm_data_t;

#endif