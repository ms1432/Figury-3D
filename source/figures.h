/*
 * figures.h
 *
 *  Created on: 13 maj 2025
 *      Author: michalstrek
 */

#ifndef FIGURES_H
#define FIGURES_H

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "lcd.h"

#define CUBE_VERTICES_COUNT 8
#define CUBE_EDGES_COUNT 12

#define PYRAMID_VERTICES_COUNT 4
#define PYRAMID_EDGES_COUNT 6

#define LCD_WIDTH  160
#define LCD_HEIGHT 128

typedef struct{
	float x, y, z;
}Vec_3;

typedef struct{
	uint16_t x, y;
}Vec_2;

typedef struct{
	Vec_3 vertices[CUBE_VERTICES_COUNT];
	Vec_2 edges[CUBE_EDGES_COUNT];
	uint16_t color;
	float angleX;
	float angleY;
	float angleZ;
}Cube;

typedef struct{
	Vec_3 vertices[PYRAMID_VERTICES_COUNT];
	Vec_2 edges[PYRAMID_EDGES_COUNT];
	uint16_t color;
	float angleX;
	float angleY;
	float angleZ;
}Pyramid;

typedef struct {
    int x, y;
    float angleX;
    uint16_t color;
}Level;

void cubeDraw(const Cube cube);

void pyramidDraw(const Pyramid pyramid);

void levelDraw(const Level level);


#endif

