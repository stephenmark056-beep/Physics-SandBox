#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <stdexcept>
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
constexpr float kCircleRadius = gfx::pixelsToMeters(24.0f, kPixelsPerMeter);
constexpr float kPolygonRadius = 0.6f;
constexpr int kTerrainPointCount = 11;
constexpr float kRemoveBelowY = 0.0f;
constexpr float kMaxFrameTime = 1.0f / 30.0f;
constexpr float kStepDt = 1.0f / 60.0f;
constexpr float kSpawnInterval = 0.25f;
constexpr float kAxisLength = 0.6f;
constexpr int kCircleSegments = 24;
constexpr float kDegreesToRadians = b2_pi / 180.0f;

struct Item {
    b2Body* body;
    float r;
    float g;
    float b;
};

Item createItem(b2World& world, b2BodyType type, const b2Vec2& position, const b2Shape& shape,
                float density, float friction, float restitution, float r, float g, float b,
                float angle = 0.0f) {
    b2BodyDef bodyDef;
    bodyDef.type = type;
    bodyDef.position = position;
    bodyDef.angle = angle;
    b2Body* body = world.CreateBody(&bodyDef);

    b2FixtureDef fixtureDef;
    fixtureDef.shape = &shape;
    fixtureDef.density = density;
    fixtureDef.friction = friction;
    fixtureDef.restitution = restitution;
    body->CreateFixture(&fixtureDef);
    return {body, r, g, b};
}

Item createRegularPolygonItem(b2World& world, const b2Vec2& position, int vertexCount) {
    b2Vec2 points[b2_maxPolygonVertices];
    for (int i = 0; i < vertexCount; ++i) {
        const float angle = (90.0f + 360.0f * static_cast<float>(i) / vertexCount) *
                            kDegreesToRadians;
        points[i].Set(kPolygonRadius * std::cos(angle), kPolygonRadius * std::sin(angle));
    }

    b2PolygonShape shape;
    if (!shape.Set(points, vertexCount)) {
        throw std::runtime_error("Failed to create a valid polygon shape");
    }
    return createItem(world, b2_dynamicBody, position, shape, 4.0f, 0.3f, 0.0f, 1.0f, 0.5f,
                      0.2f);
}

void drawBodyAxes(const b2Body& body) {
    const b2Transform transform = body.GetTransform();
    const b2Vec2 origin = transform.p;
    const b2Vec2 xAxisEnd = b2Mul(transform, b2Vec2(kAxisLength, 0.0f));
    const b2Vec2 yAxisEnd = b2Mul(transform, b2Vec2(0.0f, kAxisLength));

    const auto drawWorldLine = [&](const b2Vec2& end, float r, float g, float b) {
        gfx::drawLine(
            gfx::metersToPixels(origin.x, kPixelsPerMeter),
            gfx::metersToPixelsY(origin.y, kWindowHeight, kPixelsPerMeter),
            gfx::metersToPixels(end.x, kPixelsPerMeter),
            gfx::metersToPixelsY(end.y, kWindowHeight, kPixelsPerMeter), r, g, b);
    };

    drawWorldLine(xAxisEnd, 1.0f, 0.0f, 0.0f);
    drawWorldLine(yAxisEnd, 0.0f, 1.0f, 0.0f);
}

void drawCircle(const b2Vec2& center, float radius, float r, float g, float b) {
    for (int i = 0; i < kCircleSegments; ++i) {
        const float startAngle = 2.0f * b2_pi * static_cast<float>(i) / kCircleSegments;
        const float endAngle = 2.0f * b2_pi * static_cast<float>(i + 1) / kCircleSegments;
        const b2Vec2 start =
            center + radius * b2Vec2(std::cos(startAngle), std::sin(startAngle));
        const b2Vec2 end = center + radius * b2Vec2(std::cos(endAngle), std::sin(endAngle));
        gfx::drawLine(gfx::metersToPixels(start.x, kPixelsPerMeter),
                      gfx::metersToPixelsY(start.y, kWindowHeight, kPixelsPerMeter),
                      gfx::metersToPixels(end.x, kPixelsPerMeter),
                      gfx::metersToPixelsY(end.y, kWindowHeight, kPixelsPerMeter), r, g, b);
    }
}

void drawItem(const b2Body& body, float r, float g, float b) {
    const b2Transform transform = body.GetTransform();
    for (const b2Fixture* fixture = body.GetFixtureList(); fixture != nullptr;
         fixture = fixture->GetNext()) {
        const b2Shape* shape = fixture->GetShape();
        switch (shape->GetType()) {
            case b2Shape::e_polygon: {
                const auto* polygon = static_cast<const b2PolygonShape*>(shape);
                for (int i = 0; i < polygon->m_count; ++i) {
                    const b2Vec2 start = b2Mul(transform, polygon->m_vertices[i]);
                    const b2Vec2 end =
                        b2Mul(transform, polygon->m_vertices[(i + 1) % polygon->m_count]);
                    gfx::drawLine(
                        gfx::metersToPixels(start.x, kPixelsPerMeter),
                        gfx::metersToPixelsY(start.y, kWindowHeight, kPixelsPerMeter),
                        gfx::metersToPixels(end.x, kPixelsPerMeter),
                        gfx::metersToPixelsY(end.y, kWindowHeight, kPixelsPerMeter), r, g, b);
                }
                break;
            }
            case b2Shape::e_circle: {
                const auto* circle = static_cast<const b2CircleShape*>(shape);
                const b2Vec2 center = b2Mul(transform, circle->m_p);
                drawCircle(center, circle->m_radius, r, g, b);
                break;
            }
            case b2Shape::e_chain: {
                const auto* chain = static_cast<const b2ChainShape*>(shape);
                for (int i = 0; i < chain->m_count - 1; ++i) {
                    const b2Vec2 start = b2Mul(transform, chain->m_vertices[i]);
                    const b2Vec2 end = b2Mul(transform, chain->m_vertices[i + 1]);
                    gfx::drawLine(
                        gfx::metersToPixels(start.x, kPixelsPerMeter),
                        gfx::metersToPixelsY(start.y, kWindowHeight, kPixelsPerMeter),
                        gfx::metersToPixels(end.x, kPixelsPerMeter),
                        gfx::metersToPixelsY(end.y, kWindowHeight, kPixelsPerMeter), r, g, b);
                }
                break;
            }
            default:
                throw std::invalid_argument("drawItem does not support this shape type");
        }
    }
}

}  // namespace

int main() {
    try {
        gfx::GlfwContext glfw;
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
        gfx::Window window({.width = kWindowWidth, .height = kWindowHeight, .title = "Box2D"});
        gfx::initDraw(kWindowWidth, kWindowHeight);

        b2World world({0.0f, -9.8f});

        const b2Vec2 terrainPoints[kTerrainPointCount] = {
            {21.875f, 3.3f}, {20.0f, 3.8f},  {18.125f, 3.0f}, {16.25f, 3.6f},
            {14.375f, 2.9f}, {12.5f, 3.8f},  {10.625f, 3.1f}, {8.75f, 3.7f},
            {6.875f, 2.8f},  {5.0f, 3.5f},   {3.125f, 3.2f},
        };
        b2ChainShape terrainShape;
        terrainShape.CreateChain(terrainPoints, kTerrainPointCount, {23.75f, 2.8f},
                                 {1.25f, 2.9f});
        const Item terrain =
            createItem(world, b2_staticBody, {0.0f, 0.0f}, terrainShape, 0.0f, 0.6f, 0.0f,
                       0.65f, 0.65f, 0.65f);

        std::vector<Item> items;

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);

        double previousTime = glfwGetTime();
        float accumulator = 0.0f;
        float spawnTimer = 0.0f;
        bool wasMouseDown = false;
        bool wasKey1Down = false;
        bool wasKey2Down = false;
        bool wasKey3Down = false;
        bool wasKey4Down = false;
        bool wasKey5Down = false;

        while (!window.shouldClose()) {
            gfx::processInput(window);

            const double currentTime = glfwGetTime();
            const float elapsedTime = static_cast<float>(currentTime - previousTime);
            previousTime = currentTime;
            const float frameTime = std::min(elapsedTime, kMaxFrameTime);

            double mouseX = 0.0, mouseY = 0.0;
            window.getCursorPosition(mouseX, mouseY);
            const b2Vec2 spawnPosition{
                gfx::pixelsToMeters(static_cast<float>(mouseX), kPixelsPerMeter),
                gfx::pixelsToMetersY(static_cast<float>(mouseY), kWindowHeight, kPixelsPerMeter)};
            const auto spawnBoxAtCursor = [&]() {
                b2PolygonShape shape;
                shape.SetAsBox(kBoxHalfWidth, kBoxHalfHeight);
                items.push_back(createItem(world, b2_dynamicBody, spawnPosition, shape, 4.0f,
                                           0.3f, 0.0f, 1.0f, 0.5f, 0.2f));
            };

            const bool key1Down = window.isKeyPressed(GLFW_KEY_1);
            const bool key2Down = window.isKeyPressed(GLFW_KEY_2);
            const bool key3Down = window.isKeyPressed(GLFW_KEY_3);
            const bool key4Down = window.isKeyPressed(GLFW_KEY_4);
            const bool key5Down = window.isKeyPressed(GLFW_KEY_5);
            if (key1Down && !wasKey1Down) {
                spawnBoxAtCursor();
            }
            if (key2Down && !wasKey2Down) {
                b2CircleShape shape;
                shape.m_radius = kCircleRadius;
                items.push_back(createItem(world, b2_dynamicBody, spawnPosition, shape, 4.0f,
                                           0.3f, 0.0f, 1.0f, 0.5f, 0.2f));
            }
            if (key3Down && !wasKey3Down) {
                items.push_back(createRegularPolygonItem(world, spawnPosition, 3));
            }
            if (key4Down && !wasKey4Down) {
                items.push_back(createRegularPolygonItem(world, spawnPosition, 5));
            }
            if (key5Down && !wasKey5Down) {
                items.push_back(createRegularPolygonItem(world, spawnPosition, 6));
            }
            wasKey1Down = key1Down;
            wasKey2Down = key2Down;
            wasKey3Down = key3Down;
            wasKey4Down = key4Down;
            wasKey5Down = key5Down;

            const bool mouseDown = window.isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT);
            if (mouseDown) {
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
            for (auto item = items.begin(); item != items.end();) {
                const b2Fixture* fixture = item->body->GetFixtureList();
                if (fixture->GetAABB(0).lowerBound.y < kRemoveBelowY) {
                    world.DestroyBody(item->body);
                    item = items.erase(item);
                } else {
                    ++item;
                }
            }

            glClear(GL_COLOR_BUFFER_BIT);

            drawItem(*terrain.body, terrain.r, terrain.g, terrain.b);

            for (const Item& item : items) {
                drawItem(*item.body, item.r, item.g, item.b);
                drawBodyAxes(*item.body);
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
