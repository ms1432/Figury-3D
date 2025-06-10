/*
 * moving_average_filter.c
 *
 *  Created on: 13 maj 2025
 *      Author: michalstrek
 */
#include "moving_average_filter.h"


void initMovingAverage(MovingAverageBuffer *ma) {
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        ma->buffer[i] = 0.0f;
    }
    ma->index = 0;
}

float updateMovingAverage(MovingAverageBuffer *ma, float newValue) {
    ma->buffer[ma->index] = newValue;
    ma->index = (ma->index + 1) % SAMPLE_COUNT;

    float sum = 0.0f;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        sum += ma->buffer[i];
    }
    return sum / SAMPLE_COUNT;
}
