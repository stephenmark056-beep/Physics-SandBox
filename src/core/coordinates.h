#pragma once

namespace gfx {

constexpr float pixelsToMeters(float pixels, float pixelsPerMeter) {
    return pixels / pixelsPerMeter;
}

constexpr float metersToPixels(float meters, float pixelsPerMeter) {
    return meters * pixelsPerMeter;
}

constexpr float pixelsToMetersY(float pixelY, float screenHeight, float pixelsPerMeter) {
    return pixelsToMeters(screenHeight - pixelY, pixelsPerMeter);
}

constexpr float metersToPixelsY(float meterY, float screenHeight, float pixelsPerMeter) {
    return screenHeight - metersToPixels(meterY, pixelsPerMeter);
}

}  // namespace gfx
