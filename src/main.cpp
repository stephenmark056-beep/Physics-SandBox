#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <vector>

#include <box2d/box2d.h>

#include "core/glfw_context.h"
#include "core/input.h"
#include "core/window.h"
#include "draw.h"

namespace {

constexpr int kWindowWidth = 800;
constexpr int kWindowHeight = 600;
constexpr float kPixelsPerMeter = 32.0f;

constexpr float pixelsToMeters(float pixels) {
    return pixels / kPixelsPerMeter;
}

constexpr float metersToPixels(float meters) {
    return meters * kPixelsPerMeter;
}

// Decided to minimize the size of the box so that there will be more space for me to spam this
constexpr float kBoxHalfWidth = pixelsToMeters(30.0f);
constexpr float kBoxHalfHeight = pixelsToMeters(20.0f);
constexpr float kGroundWidth = kWindowWidth - 200.0f;
constexpr float kGroundHalfWidth = pixelsToMeters(kGroundWidth / 2.0f);
constexpr float kGroundHalfHeight = pixelsToMeters(15.0f);
constexpr float kMaxFrameTime = 1.0f / 30.0f;
constexpr float kTimeStep = 1.0f / 60.0f;

b2Body* createBox(b2World& world, b2BodyType type, const b2Vec2& position,
                  const b2Vec2& halfExtents, float density) {
    b2BodyDef bodyDef;
    bodyDef.type = type;
    bodyDef.position = position;
    b2Body* body = world.CreateBody(&bodyDef);

    b2PolygonShape shape;
    shape.SetAsBox(halfExtents.x, halfExtents.y);

    b2FixtureDef fixtureDef;
    fixtureDef.shape = &shape;
    fixtureDef.density = density;
    body->CreateFixture(&fixtureDef);
    return body;
}

}  // namespace

int main() {
    try {
        gfx::GlfwContext glfw;
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
        gfx::Window window({.width = kWindowWidth, .height = kWindowHeight, .title = "Box2D"});
        gfx::initDraw(kWindowWidth, kWindowHeight);

        b2World world({0.0f, -9.8f});

        b2Body* groundBody = createBox(
            world, b2_staticBody, {pixelsToMeters(kWindowWidth / 2.0f), kGroundHalfHeight},
            {kGroundHalfWidth, kGroundHalfHeight}, 0.0f);

        std::vector<b2Body*> boxes;
        boxes.push_back(createBox(world, b2_dynamicBody,
                                  {pixelsToMeters(kWindowWidth / 2.0f),
                                   pixelsToMeters(kWindowHeight - 100.0f)},
                                  {kBoxHalfWidth, kBoxHalfHeight}, 1.0f));

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);

        double previousTime = glfwGetTime();
        float accumulator = 0.0f;
        bool wasMouseDown = false;

        while (!window.shouldClose()) {
            gfx::processInput(window);

            double mouseX = 0.0, mouseY = 0.0;
            window.getCursorPosition(mouseX, mouseY);
            const bool mouseDown = window.isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT);
            if (mouseDown && !wasMouseDown) {
                std::cout << "click at " << mouseX << ", " << mouseY << '\n';
                boxes.push_back(createBox(
                    world, b2_dynamicBody,
                    {pixelsToMeters(static_cast<float>(mouseX)),
                     pixelsToMeters(kWindowHeight - static_cast<float>(mouseY))},
                    {kBoxHalfWidth, kBoxHalfHeight}, 1.0f));
            }
            wasMouseDown = mouseDown;

            const double currentTime = glfwGetTime();
            const float frameTime = std::min(static_cast<float>(currentTime - previousTime),
                                              kMaxFrameTime);
            previousTime = currentTime;

            accumulator += frameTime;
            while (accumulator >= kTimeStep) {
                world.Step(kTimeStep, 8, 3);
                accumulator -= kTimeStep;
            }
            for (auto box = boxes.begin(); box != boxes.end();) {
                const b2Vec2 position = (*box)->GetPosition();
                const float angle = (*box)->GetAngle();
                const float verticalExtent = std::abs(std::sin(angle)) * kBoxHalfWidth +
                                             std::abs(std::cos(angle)) * kBoxHalfHeight;
                if (position.y < -verticalExtent) {
                    world.DestroyBody(*box);
                    box = boxes.erase(box);
                } else {
                    ++box;
                }
            }

            glClear(GL_COLOR_BUFFER_BIT);

            gfx::drawBoxBody(*groundBody, kPixelsPerMeter, kWindowHeight, 0.65f, 0.65f, 0.65f);

            for (const b2Body* box : boxes) {
                gfx::drawBoxBody(*box, kPixelsPerMeter, kWindowHeight, 1.0f, 0.5f, 0.2f);
            }

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
