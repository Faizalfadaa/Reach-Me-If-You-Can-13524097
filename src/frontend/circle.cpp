#include "iostream"
#include "cmath"
#include "raylib.h"

// Create a circle for an obstacle
struct Circle {
    Vector2 center;
    float radius;
};

// Draw static circle obstacle on the canvas
void drawCircleObject(Circle obstacle) {
    DrawCircleV(obstacle.center, obstacle.radius, Color{120, 122, 128, 255});
}

// Check whether a point is inside the obstacle
bool isPointInsideCircle(Vector2 point, Circle obstacle) {
    float dx = point.x - obstacle.center.x;
    float dy = point.y - obstacle.center.y;
    float distance = sqrt(dx * dx + dy * dy);

    return distance <= obstacle.radius;
}