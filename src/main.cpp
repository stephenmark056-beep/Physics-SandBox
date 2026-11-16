#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <numbers>
#include <stdexcept>

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
constexpr float kBarrelHalfHeight = 0.125f;
constexpr float kMaxAimAngle = std::numbers::pi_v<float> / 2.0f - 0.01f;

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

void drawBody(const b2Body& body) {
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
            kGroundColor, kGroundColor, kGroundColor);
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

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);

        while (!window.shouldClose()) {
            gfx::processInput(window);

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

            glClear(GL_COLOR_BUFFER_BIT);
            drawBody(*ground);
            drawBody(*base);
            drawBarrel(aimAngle);
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
