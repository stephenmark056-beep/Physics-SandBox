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
#include "core/coordinates.h"
#include "core/input.h"
#include "core/window.h"
#include "draw.h"

namespace {

constexpr int kWindowWidth = 800;
constexpr int kWindowHeight = 600;
constexpr float kPixelsPerMeter = 32.0f;

// Decided to minimize the size of the box so that there will be more space for me to spam this
constexpr float kBoxHalfWidth = gfx::pixelsToMeters(30.0f, kPixelsPerMeter);
constexpr float kBoxHalfHeight = gfx::pixelsToMeters(20.0f, kPixelsPerMeter);
constexpr float kGroundWidth = kWindowWidth - 200.0f;
constexpr float kGroundHalfWidth = gfx::pixelsToMeters(kGroundWidth / 2.0f, kPixelsPerMeter);
constexpr float kGroundHalfHeight = gfx::pixelsToMeters(15.0f, kPixelsPerMeter);
constexpr float kRemoveBelowY = 0.0f;
constexpr float kMaxFrameTime = 1.0f / 30.0f;
constexpr float kStepDt = 1.0f / 60.0f;
constexpr float kSlideSpeed = 5.0f;
constexpr float kSpawnInterval = 0.25f;

struct Box {
    b2Body* body;
    b2Vec2 halfExtents;
    float friction;
    float restitution;
    float r;
    float g;
    float b;
};

Box createBox(b2World& world, b2BodyType type, const b2Vec2& position,
              const b2Vec2& halfExtents, float density, float friction, float restitution,
              float r, float g, float b, float angle = 0.0f) {
    b2BodyDef bodyDef;
    bodyDef.type = type;
    bodyDef.position = position;
    bodyDef.angle = angle;
    b2Body* body = world.CreateBody(&bodyDef);

    b2PolygonShape shape;
    shape.SetAsBox(halfExtents.x, halfExtents.y);

    b2FixtureDef fixtureDef;
    fixtureDef.shape = &shape;
    fixtureDef.density = density;
    fixtureDef.friction = friction;
    fixtureDef.restitution = restitution;
    body->CreateFixture(&fixtureDef);
    return {body, halfExtents, friction, restitution, r, g, b};
}

}  // namespace

int main() {
    try {
        gfx::GlfwContext glfw;
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
        gfx::Window window({.width = kWindowWidth, .height = kWindowHeight, .title = "Box2D"});
        gfx::initDraw(kWindowWidth, kWindowHeight);

        b2World world({0.0f, -9.8f});

        const Box ground = createBox(
            world, b2_staticBody,
            {gfx::pixelsToMeters(kWindowWidth / 2.0f, kPixelsPerMeter), kGroundHalfHeight},
            {kGroundHalfWidth, kGroundHalfHeight}, 0.0f, 0.6f, 0.0f, 0.65f, 0.65f, 0.65f);

        std::vector<Box> boxes;
        boxes.push_back(createBox(world, b2_dynamicBody, {4.0f, kGroundHalfHeight * 2.0f + 0.5f},
                                  {0.5f, 0.5f}, 1.0f, 0.9f, 0.0f, 0.9f, 0.3f, 0.3f));
        boxes.push_back(createBox(world, b2_dynamicBody, {7.0f, kGroundHalfHeight * 2.0f + 0.5f},
                                  {1.0f, 0.5f}, 1.0f, 0.05f, 0.0f, 0.3f, 0.5f, 0.9f));
        boxes.push_back(createBox(world, b2_dynamicBody, {14.0f, 5.0f}, {0.75f, 0.75f}, 1.0f,
                                  0.5f, 0.8f, 0.3f, 0.8f, 0.4f));
        boxes.push_back(createBox(world, b2_dynamicBody, {18.0f, 8.0f}, {0.5f, 1.0f}, 1.0f,
                                  0.5f, 0.4f, 0.9f, 0.8f, 0.3f, 0.4f));
        boxes[0].body->SetLinearVelocity({kSlideSpeed, 0.0f});
        boxes[1].body->SetLinearVelocity({kSlideSpeed, 0.0f});

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);

        double previousTime = glfwGetTime();
        float accumulator = 0.0f;
        float spawnTimer = 0.0f;
        bool wasMouseDown = false;

        while (!window.shouldClose()) {
            gfx::processInput(window);

            const double currentTime = glfwGetTime();
            const float elapsedTime = static_cast<float>(currentTime - previousTime);
            previousTime = currentTime;
            const float frameTime = std::min(elapsedTime, kMaxFrameTime);

            double mouseX = 0.0, mouseY = 0.0;
            window.getCursorPosition(mouseX, mouseY);
            const bool mouseDown = window.isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT);
            if (mouseDown) {
                const auto spawnBoxAtCursor = [&]() {
                    boxes.push_back(createBox(
                        world, b2_dynamicBody,
                        {gfx::pixelsToMeters(static_cast<float>(mouseX), kPixelsPerMeter),
                         gfx::pixelsToMetersY(static_cast<float>(mouseY), kWindowHeight,
                                              kPixelsPerMeter)},
                        {kBoxHalfWidth, kBoxHalfHeight}, 4.0f, 0.3f, 0.0f, 1.0f, 0.5f, 0.2f));
                };

                if (!wasMouseDown) {
                    spawnBoxAtCursor();
                }
                spawnTimer += frameTime;
                while (spawnTimer >= kSpawnInterval) {
                    spawnBoxAtCursor();
                    spawnTimer -= kSpawnInterval;
                }
            } else {
                spawnTimer = 0.0f;
            }
            wasMouseDown = mouseDown;

            accumulator += frameTime;
            while (accumulator >= kStepDt) {
                world.Step(kStepDt, 8, 3);
                accumulator -= kStepDt;
            }
            for (auto box = boxes.begin(); box != boxes.end();) {
                const b2Vec2 position = box->body->GetPosition();
                const float angle = box->body->GetAngle();
                const float verticalExtent = std::abs(std::sin(angle)) * box->halfExtents.x +
                                             std::abs(std::cos(angle)) * box->halfExtents.y;
                if (position.y - verticalExtent < kRemoveBelowY) {
                    world.DestroyBody(box->body);
                    box = boxes.erase(box);
                } else {
                    ++box;
                }
            }

            glClear(GL_COLOR_BUFFER_BIT);

            gfx::drawBoxBody(*ground.body, kPixelsPerMeter, kWindowHeight, ground.r, ground.g,
                             ground.b);

            for (const Box& box : boxes) {
                gfx::drawBoxBody(*box.body, kPixelsPerMeter, kWindowHeight, box.r, box.g, box.b);
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
