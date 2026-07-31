#include "raylib.h"

#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

int main() {
    const int screenWidth = 1200;
    const int screenHeight = 900;
    const int panelHeight = 350;


    InitWindow(screenWidth, screenHeight, "Reach Me If You Can");
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(RAYWHITE);

        Rectangle bottomPanel = {
            0,
            (screenHeight - panelHeight),
            (screenWidth),
            (panelHeight)
        };

        DrawRectangleRec(bottomPanel, Color{230, 235, 240, 255});
        DrawRectangleLinesEx(bottomPanel, 1, Color{180, 185, 190, 255});

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
