#ifndef CIRCLE_CPP
#define CIRCLE_CPP

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

// Compute the closest distance between a point and a robot segment
float distancePointToSegment(Vector2 point, Vector2 start, Vector2 end) {
    float sx = end.x - start.x;
    float sy = end.y - start.y;
    float len2 = sx * sx + sy * sy;

    if (len2 < 1e-6) {
        float dx = point.x - start.x;
        float dy = point.y - start.y;
        return sqrt(dx * dx + dy * dy);
    }

    float t = ((point.x - start.x) * sx + (point.y - start.y) * sy) / len2;
    if (t < 0.0) {
        t = 0.0;
    }

    if (t > 1.0) {
        t = 1.0;
    }

    Vector2 nearest = Vector2{start.x + t * sx, start.y + t * sy};
    float dx = point.x - nearest.x;
    float dy = point.y - nearest.y;

    return sqrt(dx * dx + dy * dy);
}

// Check whether a circle obstacle touches the robot arm
bool isCircleTouchingArm(Circle obstacle, Vector2 jointPositions[], int jointCount, float robotBodyRadius) {
    for (int i = 0; i < jointCount; i++) {
        float distance = distancePointToSegment(obstacle.center, jointPositions[i], jointPositions[i + 1]);
        if (distance <= obstacle.radius + robotBodyRadius) {
            return true;
        }
    }

    return false;
}

#endif
