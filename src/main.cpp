#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

#include <box2d/box2d.h>

#include "core/coordinates.h"
#include "core/glfw_context.h"
#include "core/input.h"
#include "core/window.h"
#include "draw.h"

namespace {

constexpr int kWindowWidth = 800;
constexpr int kWindowHeight = 600;
constexpr float kPixelsPerMeter = 32.0f;
constexpr float kGroundX = gfx::pixelsToMeters(kWindowWidth / 2.0f, kPixelsPerMeter);
constexpr float kGroundY = 1.0f;
constexpr float kGroundHalfWidth = gfx::pixelsToMeters(kWindowWidth / 2.0f, kPixelsPerMeter);
constexpr float kGroundHalfHeight = 0.5f;
constexpr float kPivotX = 2.0f;
constexpr float kPivotY = 2.5f;
constexpr float kBaseHalfWidth = 1.0f;
constexpr float kBaseHalfHeight = 0.5f;
constexpr float kGroundColor = 0.65f;
constexpr float kBarrelLength = 1.5f;
constexpr float kBarrelHalfHeight = 0.25f;
constexpr float kMuzzleLength = 0.2f;
constexpr float kMuzzleHalfHeight = 0.35f;
constexpr float kMuzzlePointSize = 0.2f;
constexpr float kPivotRadius = 0.3f;
constexpr float kCannonballRadius = 0.2f;
constexpr float kCannonballDensity = 3.0f;
constexpr float kMaxAimAngle = std::numbers::pi_v<float> / 2.0f - 0.01f;
constexpr float kMinPower = 20.0f;
constexpr float kMaxPower = 40.0f;
constexpr float kChargeRate = 20.0f;
constexpr float kPowerBarWidth = 200.0f;
constexpr float kPowerBarHeight = 16.0f;
constexpr float kPowerBarMargin = 12.0f;
constexpr float kTowerLeftX = 19.0f;
constexpr float kTowerHalfSize = 0.4f;
constexpr float kTowerDensity = 1.0f;
constexpr int kTowerColumns = 3;
constexpr int kTowerRows = 5;
constexpr float kKnockDistance = 0.3f;
constexpr float kMaxFrameTime = 1.0f / 30.0f;
constexpr float kStepDt = 1.0f / 60.0f;

struct TowerBlock {
    b2Body* body;
    b2Vec2 startPosition;
    bool knocked;
};

b2Body* createStaticBox(b2World& world, float x, float y, float halfWidth, float halfHeight) {
    b2BodyDef bodyDef;
    bodyDef.type = b2_staticBody;
    bodyDef.position.Set(x, y);
    b2Body* body = world.CreateBody(&bodyDef);

    b2PolygonShape shape;
    shape.SetAsBox(halfWidth, halfHeight);
    body->CreateFixture(&shape, 0.0f);
    return body;
}

b2Body* createTowerBlock(b2World& world, float x, float y) {
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.position.Set(x, y);
    b2Body* body = world.CreateBody(&bodyDef);

    b2PolygonShape shape;
    shape.SetAsBox(kTowerHalfSize, kTowerHalfSize);

    b2FixtureDef fixtureDef;
    fixtureDef.shape = &shape;
    fixtureDef.density = kTowerDensity;
    fixtureDef.friction = 0.5f;
    body->CreateFixture(&fixtureDef);
    return body;
}

b2Body* createCannonball(b2World& world, const b2Vec2& position) {
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.position = position;
    b2Body* body = world.CreateBody(&bodyDef);

    b2CircleShape shape;
    shape.m_radius = kCannonballRadius;

    b2FixtureDef fixtureDef;
    fixtureDef.shape = &shape;
    fixtureDef.density = kCannonballDensity;
    fixtureDef.friction = 0.3f;
    body->CreateFixture(&fixtureDef);
    return body;
}

void drawBody(const b2Body& body, float r = kGroundColor, float g = kGroundColor,
              float b = kGroundColor) {
    for (const b2Fixture* fixture = body.GetFixtureList(); fixture != nullptr;
         fixture = fixture->GetNext()) {
        const b2Shape* shape = fixture->GetShape();
        if (shape->GetType() != b2Shape::e_polygon) {
            throw std::invalid_argument("drawBody only supports polygon shapes");
        }

        const auto* polygon = static_cast<const b2PolygonShape*>(shape);
        b2Vec2 lowerBound = polygon->m_vertices[0];
        b2Vec2 upperBound = polygon->m_vertices[0];
        for (int i = 1; i < polygon->m_count; ++i) {
            lowerBound.x = b2Min(lowerBound.x, polygon->m_vertices[i].x);
            lowerBound.y = b2Min(lowerBound.y, polygon->m_vertices[i].y);
            upperBound.x = b2Max(upperBound.x, polygon->m_vertices[i].x);
            upperBound.y = b2Max(upperBound.y, polygon->m_vertices[i].y);
        }

        const b2Vec2 localCenter = 0.5f * (lowerBound + upperBound);
        const b2Vec2 worldCenter = b2Mul(body.GetTransform(), localCenter);
        gfx::drawBox(
            gfx::metersToPixels(worldCenter.x, kPixelsPerMeter),
            gfx::metersToPixelsY(worldCenter.y, kWindowHeight, kPixelsPerMeter),
            gfx::metersToPixels(upperBound.x - lowerBound.x, kPixelsPerMeter),
            gfx::metersToPixels(upperBound.y - lowerBound.y, kPixelsPerMeter), -body.GetAngle(),
            r, g, b);
    }
}

void drawBarrel(float angle) {
    const float centerX = kPivotX + std::cos(angle) * kBarrelLength / 2.0f;
    const float centerY = kPivotY + std::sin(angle) * kBarrelLength / 2.0f;
    gfx::drawBox(
        gfx::metersToPixels(centerX, kPixelsPerMeter),
        gfx::metersToPixelsY(centerY, kWindowHeight, kPixelsPerMeter),
        gfx::metersToPixels(kBarrelLength, kPixelsPerMeter),
        gfx::metersToPixels(kBarrelHalfHeight * 2.0f, kPixelsPerMeter), -angle, 0.25f, 0.25f,
        0.25f);

    const float muzzleCenterX =
        kPivotX + std::cos(angle) * (kBarrelLength - kMuzzleLength / 2.0f);
    const float muzzleCenterY =
        kPivotY + std::sin(angle) * (kBarrelLength - kMuzzleLength / 2.0f);
    gfx::drawBox(
        gfx::metersToPixels(muzzleCenterX, kPixelsPerMeter),
        gfx::metersToPixelsY(muzzleCenterY, kWindowHeight, kPixelsPerMeter),
        gfx::metersToPixels(kMuzzleLength, kPixelsPerMeter),
        gfx::metersToPixels(kMuzzleHalfHeight * 2.0f, kPixelsPerMeter), -angle, 0.35f, 0.35f,
        0.35f);
}

void drawMuzzlePoint(float angle) {
    const float muzzleX = kPivotX + std::cos(angle) * kBarrelLength;
    const float muzzleY = kPivotY + std::sin(angle) * kBarrelLength;
    const float size = gfx::metersToPixels(kMuzzlePointSize, kPixelsPerMeter);
    gfx::drawBox(gfx::metersToPixels(muzzleX, kPixelsPerMeter),
                 gfx::metersToPixelsY(muzzleY, kWindowHeight, kPixelsPerMeter), size, size, 0.0f,
                 1.0f, 0.9f, 0.0f);
}

void drawPowerBar(float power) {
    const float barY = kWindowHeight - kPowerBarMargin - kPowerBarHeight / 2.0f;
    const float barLeft = kPowerBarMargin;
    gfx::drawBox(barLeft + kPowerBarWidth / 2.0f, barY, kPowerBarWidth, kPowerBarHeight, 0.0f,
                 0.12f, 0.12f, 0.16f);

    const float fillFraction = (power - kMinPower) / (kMaxPower - kMinPower);
    const float fillWidth = kPowerBarWidth * fillFraction;
    if (fillWidth > 0.0f) {
        gfx::drawBox(barLeft + fillWidth / 2.0f, barY, fillWidth, kPowerBarHeight, 0.0f, 0.1f,
                     0.45f, 1.0f);
    }
}

}  // namespace

int main() {
    try {
        gfx::GlfwContext glfw;
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
        gfx::Window window({.width = kWindowWidth, .height = kWindowHeight,
                            .title = "Physics Sandbox"});
        gfx::initDraw(kWindowWidth, kWindowHeight);

        b2World world({0.0f, -9.8f});
        b2Body* ground = createStaticBox(world, kGroundX, kGroundY, kGroundHalfWidth,
                                         kGroundHalfHeight);
        b2Body* base = createStaticBox(world, kPivotX, kPivotY - kBaseHalfHeight,
                                       kBaseHalfWidth, kBaseHalfHeight);

        std::vector<TowerBlock> towerBlocks;
        std::vector<b2Body*> cannonballs;
        const float groundTopY = kGroundY + kGroundHalfHeight;
        for (int row = 0; row < kTowerRows; ++row) {
            for (int column = 0; column < kTowerColumns; ++column) {
                const float x = kTowerLeftX + column * 2.0f * kTowerHalfSize;
                const float y =
                    groundTopY + kTowerHalfSize + row * 2.0f * kTowerHalfSize;
                b2Body* body = createTowerBlock(world, x, y);
                towerBlocks.push_back({body, body->GetPosition(), false});
            }
        }

        int knockedCount = 0;
        window.setTitle("knocked: 0/" + std::to_string(static_cast<int>(towerBlocks.size())));

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        double previousTime = glfwGetTime();
        float power = kMinPower;
        float accumulator = 0.0f;
        bool wasMouseDown = false;

        while (!window.shouldClose()) {
            gfx::processInput(window);
            const bool mouseDown = window.isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT);

            const double currentTime = glfwGetTime();
            const float elapsedTime = static_cast<float>(currentTime - previousTime);
            previousTime = currentTime;
            const float frameTime = std::min(elapsedTime, kMaxFrameTime);
            if (mouseDown) {
                power = std::min(kMaxPower, power + kChargeRate * elapsedTime);
            } else {
                power = kMinPower;
            }

            accumulator += frameTime;
            while (accumulator >= kStepDt) {
                world.Step(kStepDt, 8, 3);
                accumulator -= kStepDt;
            }

            bool knockedCountChanged = false;
            for (TowerBlock& block : towerBlocks) {
                if (!block.knocked) {
                    const b2Vec2 displacement = block.body->GetPosition() - block.startPosition;
                    if (displacement.LengthSquared() >= kKnockDistance * kKnockDistance) {
                        block.knocked = true;
                        ++knockedCount;
                        knockedCountChanged = true;
                    }
                }
            }
            if (knockedCountChanged) {
                window.setTitle("knocked: " + std::to_string(knockedCount) + "/" +
                                std::to_string(static_cast<int>(towerBlocks.size())));
            }

            double mouseX = 0.0;
            double mouseY = 0.0;
            window.getCursorPosition(mouseX, mouseY);
            const float mouseWorldX =
                gfx::pixelsToMeters(static_cast<float>(mouseX), kPixelsPerMeter);
            const float mouseWorldY = gfx::pixelsToMetersY(
                static_cast<float>(mouseY), kWindowHeight, kPixelsPerMeter);
            const float dx = mouseWorldX - kPivotX;
            const float dy = mouseWorldY - kPivotY;
            const float aimAngle = std::clamp(std::atan2(dy, dx), 0.0f, kMaxAimAngle);

            if (wasMouseDown && !mouseDown) {
                const b2Vec2 muzzlePosition(
                    kPivotX + std::cos(aimAngle) * kBarrelLength,
                    kPivotY + std::sin(aimAngle) * kBarrelLength);
                cannonballs.push_back(createCannonball(world, muzzlePosition));
            }
            wasMouseDown = mouseDown;

            glClear(GL_COLOR_BUFFER_BIT);
            drawBody(*ground);
            drawBody(*base);
            for (const TowerBlock& block : towerBlocks) {
                if (block.knocked) {
                    drawBody(*block.body, 0.1f, 0.9f, 0.2f);
                } else if (block.body->IsAwake()) {
                    drawBody(*block.body, 1.0f, 0.45f, 0.1f);
                } else {
                    drawBody(*block.body, 0.55f, 0.22f, 0.05f);
                }
            }
            drawBarrel(aimAngle);
            drawMuzzlePoint(aimAngle);
            gfx::drawCircle(gfx::metersToPixels(kPivotX, kPixelsPerMeter),
                            gfx::metersToPixelsY(kPivotY, kWindowHeight, kPixelsPerMeter),
                            gfx::metersToPixels(kPivotRadius, kPixelsPerMeter), 0.45f, 0.45f,
                            0.45f);
            for (const b2Body* ball : cannonballs) {
                const b2Vec2 position = ball->GetPosition();
                gfx::drawCircle(gfx::metersToPixels(position.x, kPixelsPerMeter),
                                gfx::metersToPixelsY(position.y, kWindowHeight, kPixelsPerMeter),
                                gfx::metersToPixels(kCannonballRadius, kPixelsPerMeter), 0.1f,
                                0.1f, 0.1f);
            }
            drawPowerBar(power);
            window.swapBuffers();
            window.pollEvents();
        }
        gfx::shutdownDraw();
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
