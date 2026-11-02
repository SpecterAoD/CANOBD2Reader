#pragma once
#include <TFT_eSPI.h>

namespace DisplayCanvas {
extern TFT_eSPI tft;
extern TFT_eSprite frame;
constexpr int Width = 320;
constexpr int Height = 170;
constexpr int StatusBarHeight = 18;
}

namespace DisplayRenderer {
// Draws pages and widgets into the existing sprite, then pushes one frame.
void render();
}
