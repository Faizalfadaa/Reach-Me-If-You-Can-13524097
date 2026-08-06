#include "raylib.h"
#include "cmath"
#include "../backend/invers/inverseKinematics.cpp"
#include "../backend/bridge/bridge.hpp"
#include "circle.cpp"

#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

int main() {
    const int screenWidth = 1200;
    const int screenHeight = 900;
    const int panelHeight = 350;
    const int canvasHeight = screenHeight - panelHeight;
    const int animationDelay = 2;

    // Main GUI inputs required by the task
    int jointCount = 2;
    int segmentLengths[5] = {120, 90, 80, 70, 60};
    float jointAngles[5] = {0, 0, 0, 0, 0};
    Vector2 jointPositions[6] = {};
    int minAngles[5] = {-180, -180, -180, -180, -180};
    int maxAngles[5] = {180, 180, 180, 180, 180};

    bool lengthEditMode[5] = {};
    bool minAngleEditMode[5] = {};
    bool maxAngleEditMode[5] = {};

    // Target of the inverse kinematics
    Vector2 target; 
    bool hasTarget = false;

    // Circle obstacles
    vector<Circle> obstacles;
    Vector2 circleTarget;
    bool isObjectButtonClicked = false;
    bool isDeleteObjectButtonClicked = false;
    bool isAlgorithmSelectionShown = false;

    // Inverse Kinematics data
    vector<IKResult> ikResult;
    IKStatus ikStatus = IK_RUNNING;
    bool hasIKStatus = false;

    // for animation purposes
    bool isRunning = false;
    int animationFrame = 0;
    int animationDelayCounter = 0;

    // Initialize the GUI window
    InitWindow(screenWidth, screenHeight, "Reach Me If You Can");
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(RAYWHITE);

        // Initialize the first joint segment
        jointPositions[0] = Vector2{screenWidth * 0.5, canvasHeight - 70};
        Vector2 basePosition = jointPositions[0];

        float displayAngles[5] = {};
        for (int i = 0; i < jointCount; i++) {
            displayAngles[i] = jointAngles[i];
        }

        // When animation is running, draw the current IK iteration result
        if (isRunning && !ikResult.empty()) {
            if (animationFrame >= ikResult.size()) {
                animationFrame = ikResult.size() - 1;
                isRunning = false;
            }

            int resultAngleCount = ikResult[animationFrame].angles.size();
            for (int i = 0; i < jointCount && i < resultAngleCount; i++) {
                displayAngles[i] = ikResult[animationFrame].angles[i] / DEG2RAD;
            }

            animationDelayCounter++;
            if (animationDelayCounter >= animationDelay) {
                animationDelayCounter = 0;
                animationFrame++;
            }
        }

        float totalAngle = 0;

        // For each joint segment, update every position of the joint using polar to cartesian formula
        for (int i = 0; i < jointCount; i++) {
            totalAngle += displayAngles[i] * DEG2RAD;
            jointPositions[i + 1] = Vector2{
                jointPositions[i].x + cos(totalAngle) * segmentLengths[i],
                jointPositions[i].y - sin(totalAngle) * segmentLengths[i]
            };
        }

        // Draw every circle obstacle
        for (int i = 0; i < obstacles.size(); i++) {
            drawCircleObject(obstacles[i]);
        }

        for (int i = 0; i < jointCount; i++) {
            // Segment body
            DrawLineEx(jointPositions[i], jointPositions[i + 1], 10, Color{82, 135, 184, 255});

            // Joint on the start of each segment
            DrawCircleV(jointPositions[i], 13, Color{45, 48, 52, 255});
        }

        // End effector marker
        DrawCircleV(jointPositions[jointCount], 13, Color{210, 72, 72, 255});

        // Bottom panel
        Rectangle bottomPanel = {0, canvasHeight, screenWidth, panelHeight};

        // Bottom panel background and outline
        DrawRectangleRec(bottomPanel, Color{230, 235, 240, 255});
        DrawRectangleLinesEx(bottomPanel, 1, Color{180, 185, 190, 255});

        // Shared layout positions for the panel contents
        const float panelY = screenHeight - panelHeight;
        const float leftPanelWidth = 300;
        const float padding = 34;
        const float dividerX = leftPanelWidth;

        // Vertical divider between left and right panel sections
        DrawLineEx(Vector2{dividerX, panelY}, Vector2{dividerX, screenHeight}, 2.0, Color{180, 185, 190, 255});

        // Left section bottom panel for number of joints
        DrawText("Jumlah Segmen", 36, panelY + 22, 28, Color{45, 48, 52, 255});
        DrawLineEx(Vector2{36, panelY + 58}, Vector2{230, panelY + 58}, 2.0, Color{45, 48, 52, 255});
        DrawText("N", 36, panelY + 82, 30, Color{45, 48, 52, 255});

        float jointSliderValue = jointCount;
        GuiSliderBar(Rectangle{86, panelY + 88, 120, 20}, nullptr, nullptr, &jointSliderValue, 2, 5);

        jointCount = jointSliderValue + 0.5;
        DrawText(TextFormat("%d", jointCount), 230, panelY + 85, 22, Color{65, 68, 72, 255});

        // Right section bottom panel for segment input controls
        DrawText("Input Segmen", dividerX + padding, panelY + 22, 28, Color{45, 48, 52, 255});
        DrawLineEx(Vector2{dividerX + padding, panelY + 58}, Vector2{dividerX + padding + 230, panelY + 58}, 2, Color{45, 48, 52, 255});
        DrawText("Panjang", dividerX + 150, panelY + 80, 16, Color{65, 68, 72, 255});
        DrawText("Sudut Awal", dividerX + 260, panelY + 80, 16, Color{65, 68, 72, 255});
        DrawText("Min", dividerX + 665, panelY + 80, 16, Color{65, 68, 72, 255});
        DrawText("Max", dividerX + 765, panelY + 80, 16, Color{65, 68, 72, 255});
        
        bool isJacobianButtonClicked = false;
        bool isDLSButtonClicked = false;

        // Button to show inverse kinematics algorithm options
        if (!isAlgorithmSelectionShown) {
            if (GuiButton(Rectangle{36, panelY + 245, 130, 34}, "START")) {
                isAlgorithmSelectionShown = true;
            }
        } else {
            if (GuiButton(Rectangle{36, panelY + 245, 110, 34}, "JACOBIAN")) {
                isJacobianButtonClicked = true;
                isAlgorithmSelectionShown = false;
            }

            if (GuiButton(Rectangle{156, panelY + 245, 110, 34}, "DLS")) {
                isDLSButtonClicked = true;
                isAlgorithmSelectionShown = false;
            }
        }

        // Start the inverse kinematics using the selected algorithm
        if (isJacobianButtonClicked || isDLSButtonClicked) {
            if(hasTarget) {
                Matriks convertedTarget = convertTarget(target.x, target.y, basePosition.x, basePosition.y);
                vector<double> convertedLengths = convertLengths(segmentLengths, jointCount);
                vector<double> convertedAngles = convertAngles(jointAngles, jointCount);
                vector<double> convertedMinAngles = convertMinAngles(minAngles, jointCount);
                vector<double> convertedMaxAngles = convertMaxAngles(maxAngles, jointCount);
                vector<Obstacle> convertedObstacles = convertCircles(obstacles, basePosition.x, basePosition.y);

                // Start the inverse kinematics
                if (isJacobianButtonClicked) {
                    ikResult = jacobianTranspose(convertedLengths, convertedAngles, convertedTarget, convertedMinAngles, convertedMaxAngles, convertedObstacles);
                } else {
                    ikResult = DampedLeastSquares(convertedLengths, convertedAngles, convertedTarget, convertedMinAngles, convertedMaxAngles, convertedObstacles);
                }

                if (!ikResult.empty()) {
                    ikStatus = ikResult.back().status;
                    hasIKStatus = true;
                }
                
                // Start the animation
                animationFrame = 0;
                animationDelayCounter = 0;
                if (ikStatus == IK_UNREACHABLE_DISTANCE) {
                    isRunning = false;
                } else {
                    isRunning = !ikResult.empty();
                }
            }
        }

        // Button to add a new circle obstacle
        if (GuiButton(Rectangle{36, panelY + 290, 110, 34}, "ADD OBJECT")) {
            isObjectButtonClicked = true;
            isDeleteObjectButtonClicked = false;
        }

        // Button to delete a circle obstacle
        if (GuiButton(Rectangle{156, panelY + 290, 110, 34}, "DELETE OBJECT")) {
            isDeleteObjectButtonClicked = true;
            isObjectButtonClicked = false;
        }

        // Segment rows shown according to current joint count
        for (int i = 0; i < jointCount; i++) {
            const float rowY = panelY + 110 + i * 46;

            // Clamp user inputs to valid ranges before drawing controls
            if (segmentLengths[i] < 10) {
                segmentLengths[i] = 10;
            }

            if (segmentLengths[i] > 250) {
                segmentLengths[i] = 250;
            }

            if (minAngles[i] < -180) {
                minAngles[i] = -180;
            }

            if (maxAngles[i] > 180) {
                maxAngles[i] = 180;
            }

            if (minAngles[i] > maxAngles[i]) {
                minAngles[i] = maxAngles[i];
            }

            if (jointAngles[i] < minAngles[i]) {
                jointAngles[i] = minAngles[i];
            }

            if (jointAngles[i] > maxAngles[i]) {
                jointAngles[i] = maxAngles[i];
            }

            // Segment label and length input
            DrawText(TextFormat("Segmen %d", i + 1), (dividerX + padding), (rowY), 22, Color{45, 48, 52, 255});
            GuiValueBox(Rectangle{dividerX + 150, rowY - 3, 78, 28}, nullptr, &segmentLengths[i], 10, 250, lengthEditMode[i]);
            if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && CheckCollisionPointRec(GetMousePosition(), Rectangle{dividerX + 150, rowY - 3, 78, 28})) {
                lengthEditMode[i] = !lengthEditMode[i];
            }

            // Joint angle input that is constrained by min and max angle
            DrawText(TextFormat("%.0f", jointAngles[i]), dividerX + 250, rowY + 3, 18, Color{65, 68, 72, 255});
            GuiSliderBar(Rectangle{dividerX + 300, rowY + 4, 320, 20}, nullptr, nullptr, &jointAngles[i], minAngles[i], maxAngles[i]);

            // Minimum angle constraint input
            GuiValueBox(Rectangle{dividerX + 665, rowY - 3, 70, 28}, nullptr, &minAngles[i], -180, maxAngles[i], minAngleEditMode[i]);
            if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && CheckCollisionPointRec(GetMousePosition(), Rectangle{dividerX + 665, rowY - 3, 70, 28})) {
                minAngleEditMode[i] = !minAngleEditMode[i];
            }

            // Maximum angle constraint input
            GuiValueBox(Rectangle{dividerX + 765, rowY - 3, 70, 28}, nullptr, &maxAngles[i], minAngles[i], 180, maxAngleEditMode[i]);
            if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && CheckCollisionPointRec(GetMousePosition(), Rectangle{dividerX + 765, rowY - 3, 70, 28})) {
                maxAngleEditMode[i] = !maxAngleEditMode[i];
            }
        }

        // Input target position using click on canvas
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            Vector2 mousePosition = GetMousePosition();
            bool isTargetInsideObstacle = false;
            bool isObjectDeleted = false;

            // Delete object if delete button is clicked
            if(mousePosition.y < canvasHeight && isDeleteObjectButtonClicked) {
                for (int i = 0; i < obstacles.size(); i++) {
                    if (isPointInsideCircle(mousePosition, obstacles[i])) {
                        obstacles.erase(obstacles.begin() + i);
                        isObjectDeleted = true;
                        isDeleteObjectButtonClicked = false;
                        break;
                    }
                }
            }

            // Check every object if we put the target inside the obstacle
            for (int i = 0; i < obstacles.size(); i++) {
                if (isPointInsideCircle(mousePosition, obstacles[i])) {
                    isTargetInsideObstacle = true;
                }
            }

            if(mousePosition.y < canvasHeight && !isTargetInsideObstacle && isObjectButtonClicked && !isObjectDeleted) {
                circleTarget = mousePosition;
                isObjectButtonClicked = false;
                obstacles.push_back(Circle{circleTarget, 45});

            } else if (mousePosition.y < canvasHeight && !isTargetInsideObstacle && !isObjectButtonClicked && !isDeleteObjectButtonClicked && !isObjectDeleted) {
                target = mousePosition;
                hasTarget = true;
            }
        }

        // Draw the target point
        if (hasTarget) {
            float targetBackendX = target.x - basePosition.x;
            float targetBackendY = basePosition.y - target.y;

            DrawText(TextFormat("(%.0f, %.0f)", targetBackendX, targetBackendY), target.x - 38, target.y - 32, 18, Color{45, 48, 52, 255});
            DrawCircleV(target, 13, Color{45, 48, 52, 255});
        }

        // Draw inverse kinematics status
        if (hasIKStatus) {
            if (ikStatus == IK_REACHED) {
                DrawText("Target reached", 24, 20, 20, Color{40, 140, 70, 255});
            } else if (ikStatus == IK_NOT_REACHED) {
                DrawText("Target not reached", 24, 20, 20, Color{210, 72, 72, 255});
            } else if (ikStatus == IK_UNREACHABLE_DISTANCE) {
                DrawText("Target unreachable", 24, 20, 20, Color{210, 72, 72, 255});
            }

            if (!ikResult.empty()) {
                int currentIteration = animationFrame + 1;
                int totalIteration = ikResult.size();

                if (currentIteration > totalIteration) {
                    currentIteration = totalIteration;
                }

                DrawText(TextFormat("Iterasi: %d / %d", currentIteration, totalIteration), 24, 44, 20, Color{65, 68, 72, 255});
            }
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
