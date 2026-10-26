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
constexpr float kPixelsPerMeter = 30.0f;
// Decided to minimize the size of the box so that there will be more space for me to spam this
constexpr float kBoxHalfWidth = 30.0f / kPixelsPerMeter;
constexpr float kBoxHalfHeight = 20.0f / kPixelsPerMeter;
constexpr float kGroundWidth = kWindowWidth - 200.0f;
constexpr float kGroundHalfWidth = kGroundWidth / (2.0f * kPixelsPerMeter);
constexpr float kGroundHalfHeight = 15.0f / kPixelsPerMeter;
constexpr float kMaxFrameTime = 1.0f / 30.0f;
constexpr float kTimeStep = 1.0f / 60.0f;

b2Body* createDynamicBox(b2World& world, float screenX, float screenY) {
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.position.Set(screenX / kPixelsPerMeter,
                         (kWindowHeight - screenY) / kPixelsPerMeter);
    b2Body* body = world.CreateBody(&bodyDef);

    b2PolygonShape shape;
    shape.SetAsBox(kBoxHalfWidth, kBoxHalfHeight);
    body->CreateFixture(&shape, 1.0f);
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

        b2BodyDef groundBodyDef;
        groundBodyDef.position.Set(kWindowWidth / (2.0f * kPixelsPerMeter), kGroundHalfHeight);
        b2Body* groundBody = world.CreateBody(&groundBodyDef);
        b2PolygonShape groundShape;
        groundShape.SetAsBox(kGroundHalfWidth, kGroundHalfHeight);
        groundBody->CreateFixture(&groundShape, 0.0f);

        std::vector<b2Body*> boxes;
        boxes.push_back(createDynamicBox(world, kWindowWidth / 2.0f, 100.0f));

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
                boxes.push_back(createDynamicBox(world, static_cast<float>(mouseX),
                                                 static_cast<float>(mouseY)));
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

            gfx::drawBox(kWindowWidth / 2.0f, kWindowHeight - kGroundHalfHeight * kPixelsPerMeter,
                         kGroundWidth, kGroundHalfHeight * 2.0f, 0.0f, 0.65f, 0.65f, 0.65f);

            for (const b2Body* box : boxes) {
                const b2Vec2 position = box->GetPosition();
                gfx::drawBox(position.x * kPixelsPerMeter,
                             kWindowHeight - position.y * kPixelsPerMeter,
                             kBoxHalfWidth * 2.0f * kPixelsPerMeter,
                             kBoxHalfHeight * 2.0f * kPixelsPerMeter, -box->GetAngle(), 1.0f,
                             0.5f, 0.2f);
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
