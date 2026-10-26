#pragma once

#include <box2d/box2d.h>

namespace gfx {

void initDraw(int screenWidth, int screenHeight);
void shutdownDraw();

void drawBox(float x, float y, float width, float height, float angleRadians, float r, float g,
             float b);
void drawBoxBody(const b2Body& body, float pixelsPerMeter, float screenHeight, float r, float g,
                 float b);

}  // namespace gfx
