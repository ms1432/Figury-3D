/*
 * moving_average_filter.h
 *
 *  Created on: 13 maj 2025
 *      Author: michalstrek
 */

#ifndef MOVING_AVERAGE_FILTER_H_
#define MOVING_AVERAGE_FILTER_H_

#include <stdint.h>

#define SAMPLE_COUNT 10

typedef struct {
    float buffer[SAMPLE_COUNT];
    uint8_t index;
} MovingAverageBuffer;

void initMovingAverage(MovingAverageBuffer *ma);
float updateMovingAverage(MovingAverageBuffer *ma, float newValue);

#endif /* MOVING_AVERAGE_FILTER_H_ */
