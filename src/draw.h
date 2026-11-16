#pragma once

namespace gfx {

void initDraw(int screenWidth, int screenHeight);
void shutdownDraw();

void drawBox(float centerX, float centerY, float width, float height, float angle, float r, float g,
             float b);
void drawLine(float x1, float y1, float x2, float y2, float r, float g, float b);

}  // namespace gfx
