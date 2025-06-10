#include <figures.h>

void rotateY(Vec_3 *v, const float angle){
	float cosA = cos(angle);
	float sinA = sin(angle);
	float x = v->x * cosA - v->z * sinA;
	float z = v->x * sinA + v->z * cosA;
	v->x = x;
	v->z = z;

	return ;
}

void rotateX(Vec_3 *v, const float angle) {
	float cosA = cos(angle);
	float sinA = sin(angle);
	float y = v->y * cosA - v->z * sinA;
	float z = v->y * sinA + v->z * cosA;
	v->y = y;
	v->z = z;

	return ;
}

void rotateZ(Vec_3 *v, const float angle) {
    float cosA = cos(angle);
    float sinA = sin(angle);
    float x = v->x * cosA - v->y * sinA;
    float y = v->x * sinA + v->y * cosA;
    v->x = x;
    v->y = y;

    return;
}

void cubeDraw(const Cube cube){
    Vec_3 transformedVerticies[CUBE_VERTICES_COUNT];

    for (int i = 0; i < CUBE_VERTICES_COUNT; i++) {
        transformedVerticies[i] = cube.vertices[i];
    }

    for(int i = 0; i < CUBE_VERTICES_COUNT; i++){
        rotateY(&transformedVerticies[i], cube.angleY);
        rotateX(&transformedVerticies[i], cube.angleX);
        rotateZ(&transformedVerticies[i], cube.angleZ);
    }

    for (int i = 0; i < CUBE_EDGES_COUNT; i++){
        Vec_3 p1 = transformedVerticies[cube.edges[i].x];
        Vec_3 p2 = transformedVerticies[cube.edges[i].y];

        uint16_t x1 = p1.x * 40 + LCD_WIDTH / 2;
        uint16_t y1 = p1.y * 40 + LCD_HEIGHT / 2;

        uint16_t x2 = p2.x * 40 + LCD_WIDTH / 2;
        uint16_t y2 = p2.y * 40 + LCD_HEIGHT / 2;

        LCD_Draw_Line(x1, y1, x2, y2, cube.color);
    }

}

void pyramidDraw(const Pyramid pyramid) {
    Vec_3 transformedVertices[PYRAMID_VERTICES_COUNT];

    Vec_3 center = {0, 0, 0};
    for (int i = 0; i < PYRAMID_VERTICES_COUNT; i++) {
        center.x += pyramid.vertices[i].x;
        center.y += pyramid.vertices[i].y;
        center.z += pyramid.vertices[i].z;
    }
    center.x /= PYRAMID_VERTICES_COUNT;
    center.y /= PYRAMID_VERTICES_COUNT;
    center.z /= PYRAMID_VERTICES_COUNT;

    for (int i = 0; i < PYRAMID_VERTICES_COUNT; i++) {
        transformedVertices[i] = pyramid.vertices[i];

        transformedVertices[i].x -= center.x;
        transformedVertices[i].y -= center.y;
        transformedVertices[i].z -= center.z;

        rotateY(&transformedVertices[i], pyramid.angleY);
        rotateX(&transformedVertices[i], pyramid.angleX);
        rotateZ(&transformedVertices[i], pyramid.angleZ);

        transformedVertices[i].x += center.x;
        transformedVertices[i].y += center.y;
        transformedVertices[i].z += center.z;
    }

    for (int i = 0; i < PYRAMID_EDGES_COUNT; i++) {
        Vec_3 p1 = transformedVertices[pyramid.edges[i].x];
        Vec_3 p2 = transformedVertices[pyramid.edges[i].y];

        uint16_t x1 = p1.x * 40 + LCD_WIDTH / 2;
        uint16_t y1 = p1.y * 40 + LCD_HEIGHT / 2;

        uint16_t x2 = p2.x * 40 + LCD_WIDTH / 2;
        uint16_t y2 = p2.y * 40 + LCD_HEIGHT / 2;

        LCD_Draw_Line(x1, y1, x2, y2, pyramid.color);
    }
}

void levelDraw(const Level level) {
    float centerX = LCD_WIDTH / 2.0f;
    float centerY = LCD_HEIGHT / 2.0f;

    float lineLength = sqrtf(LCD_WIDTH * LCD_WIDTH + LCD_HEIGHT * LCD_HEIGHT);

    float halfLength = lineLength / 2.0f;

    float x1 = -halfLength;
    float y1 = 0;
    float x2 = halfLength;
    float y2 = 0;

    float angle = level.angleX;
    float cosAngle = cosf(angle);
    float sinAngle = sinf(angle);

    float rotX1 = x1 * cosAngle - y1 * sinAngle;
    float rotY1 = x1 * sinAngle + y1 * cosAngle;

    float rotX2 = x2 * cosAngle - y2 * sinAngle;
    float rotY2 = x2 * sinAngle + y2 * cosAngle;

    uint16_t screenX1 = (uint16_t)(rotX1 + centerX);
    uint16_t screenY1 = (uint16_t)(rotY1 + centerY);
    uint16_t screenX2 = (uint16_t)(rotX2 + centerX);
    uint16_t screenY2 = (uint16_t)(rotY2 + centerY);

    if (screenX1 >= LCD_WIDTH) screenX1 = LCD_WIDTH - 1;
    if (screenY1 >= LCD_HEIGHT) screenY1 = LCD_HEIGHT - 1;
    if (screenX2 >= LCD_WIDTH) screenX2 = LCD_WIDTH - 1;
    if (screenY2 >= LCD_HEIGHT) screenY2 = LCD_HEIGHT - 1;

    LCD_Draw_Line(screenX1, screenY1, screenX2, screenY2, level.color);
}



