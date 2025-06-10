#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include <time.h>
#include "FreeRTOS.h"
#include "task.h"
#include "fsl_device_registers.h"
#include <stdbool.h>

#include "icm20948.h"
#include "lcd.h"
#include "figures.h"
#include "moving_average_filter.h"

#ifndef MSEC_TO_TICK
#define MSEC_TO_TICK(msec) ((uint32_t)(msec) * (uint32_t)configTICK_RATE_HZ / 1000uL)
#endif

#define DELAY 10

enum Figure{
	CUBE,
	PYRAMID,
	LEVEL,
	FIGURES_COUNT
};

int currentFigure = CUBE;

int16_t rawData[10];

IMU_EN_SENSOR_TYPE enMotionSensorType;
IMU_ST_ANGLES_DATA stAngles;
IMU_ST_SENSOR_DATA stGyroRawData;
IMU_ST_SENSOR_DATA stAccelRawData;
IMU_ST_SENSOR_DATA stMagnRawData;

Cube cube = {
		.vertices = {
				{-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
				{-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}
		},
		.edges = {
				{0, 1}, {1, 2}, {2, 3}, {3, 0},
				{4, 5}, {5, 6}, {6, 7}, {7, 4},
				{0, 4}, {1, 5}, {2, 6}, {3, 7}
		},
		.color = 0xFFFF,
};

Pyramid pyramid = {
		.vertices = {
				{-1, -0.577, -0.816}, {1, -0.577, -0.816}, {0, 1.155, -0.816},
				{0, 0, 0.816}
		},
		.edges = {
				{0, 1}, {1, 2}, {2, 0},
				{0, 3}, {1, 3}, {2, 3}
		},
		.color = 0xFFFF,
};

Level level = {
    .x = 0,
    .y = LCD_HEIGHT / 2,
    .angleX = 0.2f,
    .color = 0xFFFF
};

void drawTask(void *pvParameters) {
	float angleX = 0.0f, angleY = 0.0f, angleZ = 0.0f;
	float dt; // Variable for dynamic delta time
	MovingAverageBuffer maX, maY, maZ;
	initMovingAverage(&maX);
	initMovingAverage(&maY);
	initMovingAverage(&maZ);

	bool sw2_0 = 1, sw2_1 = 1;

    TickType_t previous_tick_count = xTaskGetTickCount(); // Get initial tick count

	while (1) {
        TickType_t current_tick_count = xTaskGetTickCount();
        dt = (float)(current_tick_count - previous_tick_count) / configTICK_RATE_HZ;
        previous_tick_count = current_tick_count;

		sw2_1 = sw2_0;
		sw2_0 = GPIO_PinRead(BOARD_INITBUTTONSPINS_SW2_GPIO, BOARD_INITBUTTONSPINS_SW2_GPIO_PIN);

		if (sw2_0 == 0 && sw2_1 == 1) {
			currentFigure = (currentFigure + 1) % FIGURES_COUNT;
		}

		imuDataGet2(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);

		float gyroX = (stGyroRawData.s16X / 131.0) * (3.1415926f) * dt;
		float gyroY = (stGyroRawData.s16Y / 131.0) * (3.1415926f) * dt;
		float gyroZ = (stGyroRawData.s16Z / 131.0) * (3.1415926f) * dt;

		float filteredGyroX = updateMovingAverage(&maX, gyroX);
		float filteredGyroY = updateMovingAverage(&maY, gyroY);
		float filteredGyroZ = updateMovingAverage(&maZ, gyroZ);

		angleX += filteredGyroX * dt;
		angleY += filteredGyroY * dt;
		angleZ += filteredGyroZ * dt;

		LCD_Clear(0x00);

		if (currentFigure == CUBE) {
			cube.angleX = angleX;
			cube.angleY = angleY;
			cube.angleZ = angleZ;
			cubeDraw(cube);
		} else if (currentFigure == PYRAMID) {
			pyramid.angleX = angleX;
			pyramid.angleY = angleY;
			pyramid.angleZ = angleZ;
			pyramidDraw(pyramid);
		}else if (currentFigure == LEVEL) {
			level.angleX = angleX;
			levelDraw(level);
		}

		LCD_GramRefresh();
		vTaskDelay(MSEC_TO_TICK(DELAY));
	}
}

int main(void) {
	/* Inicjalizacja sprzętu */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();

#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
	BOARD_InitDebugConsole();
	PRINTF("Run\r\n");
	LCD_Init(LP_FLEXCOMM0_PERIPHERAL);
#endif
	LCD_Init(LP_FLEXCOMM0_PERIPHERAL);
	imuInit(LP_FLEXCOMM2_PERIPHERAL, &enMotionSensorType);


	if(IMU_EN_SENSOR_TYPE_ICM20948 == enMotionSensorType)
	{
		PRINTF("Motion sersor is ICM-20948\r\n" );
	}
	else
	{
		PRINTF("Motion sersor NULL\r\n");
	}

	if (xTaskCreate(drawTask, "DrawTask", 2048, NULL, 1, NULL) != pdPASS) {
		PRINTF("DrawTask creation failed!\r\n");
	}

	vTaskStartScheduler();

}
