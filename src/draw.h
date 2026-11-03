#pragma once

#include <box2d/box2d.h>

namespace gfx {

void initDraw(int screenWidth, int screenHeight);
void shutdownDraw();

void drawLine(float x1, float y1, float x2, float y2, float r, float g, float b);

}  // namespace gfx
