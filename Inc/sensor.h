/*
 * sensor.h
 *
 *  Created on: Jul 24, 2025
 *      Author: jamesg13
 */

// sensor.h
#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

void sensor_init(void);
uint32_t get_distance_cm(void);

#endif // SENSOR_H
