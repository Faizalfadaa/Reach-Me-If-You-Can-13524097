#include "raylib.h"

#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

int main() {
    const int screenWidth = 1200;
    const int screenHeight = 900;
    const int panelHeight = 350;

    int jointCount = 2;
    float segmentLengths[5] = {120.0f, 90.0f, 80.0f, 70.0f, 60.0f};

    // Initialize the GUI window
    InitWindow(screenWidth, screenHeight, "Reach Me If You Can");
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(RAYWHITE);

        // Bottom panel
        Rectangle bottomPanel = {0, (screenHeight - panelHeight), (screenWidth), (panelHeight)};

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

        // Right section bottom panel for segment length controls
        DrawText("Panjang Segmen", (dividerX + padding), (panelY + 22), 28, Color{45, 48, 52, 255});
        DrawLineEx(Vector2{dividerX + padding, panelY + 58}, Vector2{dividerX + padding + 230, panelY + 58}, 2, Color{45, 48, 52, 255});

        // Segment rows shown according to current joint count
        for (int i = 0; i < jointCount; i++) {
            const float rowY = panelY + 82 + i * 50;

            DrawText(TextFormat("Segmen %d", i + 1), (dividerX + padding), (rowY), 24, Color{45, 48, 52, 255});
            DrawText(TextFormat("%.1f", segmentLengths[i]), (dividerX + 210), (rowY + 3), 20, Color{65, 68, 72, 255});

            GuiSliderBar(Rectangle{dividerX + 285, rowY + 7, 430, 20}, nullptr, nullptr, &segmentLengths[i], 10, 250);
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
