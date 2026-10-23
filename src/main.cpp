#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>

#include <box2d/box2d.h>

#include "core/glfw_context.h"
#include "core/input.h"
#include "core/window.h"
#include "draw.h"

namespace {

constexpr int kWindowWidth = 800;
constexpr int kWindowHeight = 600;
constexpr float kPixelsPerMeter = 30.0f;
constexpr float kBoxHalfWidth = 60.0f / kPixelsPerMeter;
constexpr float kBoxHalfHeight = 40.0f / kPixelsPerMeter;
constexpr float kTimeStep = 1.0f / 60.0f;

}  // namespace

int main() {
    try {
        gfx::GlfwContext glfw;
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
        gfx::Window window({.width = kWindowWidth, .height = kWindowHeight, .title = "Box2D"});
        gfx::initDraw(kWindowWidth, kWindowHeight);

        b2World world({0.0f, -9.8f});

        b2BodyDef boxBodyDef;
        boxBodyDef.type = b2_dynamicBody;
        boxBodyDef.position.Set(kWindowWidth / (2.0f * kPixelsPerMeter),
                                (kWindowHeight - 100.0f) / kPixelsPerMeter);
        b2Body* boxBody = world.CreateBody(&boxBodyDef);
        b2PolygonShape boxShape;
        boxShape.SetAsBox(kBoxHalfWidth, kBoxHalfHeight);
        boxBody->CreateFixture(&boxShape, 1.0f);

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
            }
            wasMouseDown = mouseDown;

            const double currentTime = glfwGetTime();
            const float frameTime = static_cast<float>(currentTime - previousTime);
            previousTime = currentTime;
            accumulator += frameTime < 0.25f ? frameTime : 0.25f;

            while (accumulator >= kTimeStep) {
                world.Step(kTimeStep, 8, 3);
                if (boxBody != nullptr) {
                    const b2Vec2 position = boxBody->GetPosition();
                    const float angle = boxBody->GetAngle();
                    const float verticalExtent =
                        std::abs(std::sin(angle)) * kBoxHalfWidth +
                        std::abs(std::cos(angle)) * kBoxHalfHeight;
                    if (position.y < -verticalExtent) {
                        world.DestroyBody(boxBody);
                        boxBody = nullptr;
                    }
                }
                accumulator -= kTimeStep;
            }

            glClear(GL_COLOR_BUFFER_BIT);

            if (boxBody != nullptr) {
                const b2Vec2 position = boxBody->GetPosition();
                gfx::drawBox(position.x * kPixelsPerMeter,
                             kWindowHeight - position.y * kPixelsPerMeter, 120.0f, 80.0f,
                             -boxBody->GetAngle(), 1.0f, 0.5f, 0.2f);
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
