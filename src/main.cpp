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
#include <utility>
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
constexpr float kWorldWidth = kWindowWidth / kPixelsPerMeter;
constexpr float kWorldHeight = kWindowHeight / kPixelsPerMeter;
constexpr float kOutOfBoundsZoneThickness = 1.0f;
constexpr float kArcTimeStep = 0.02f;
constexpr float kArcDotRadius = 0.06f;
constexpr int kMaxArcDots = 500;
constexpr float kMaxAimAngle = std::numbers::pi_v<float> / 2.0f - 0.01f;
constexpr float kMinPower = 20.0f;
constexpr float kMaxPower = 120.0f;
constexpr float kChargeRate = 50.0f;
constexpr float kPowerBarWidth = 200.0f;
constexpr float kPowerBarHeight = 16.0f;
constexpr float kPowerBarMargin = 12.0f;
constexpr int kMaxShots = 5;
constexpr float kAmmoSquareSize = 16.0f;
constexpr float kAmmoSquareSpacing = 24.0f;
constexpr float kAmmoSquareStartX = 24.0f;
constexpr float kAmmoSquareY = 24.0f;
constexpr float kTowerLeftX = 19.0f;
constexpr float kTowerHalfSize = 0.4f;
constexpr float kTowerDensity = 1.0f;
constexpr int kTowerColumns = 3;
constexpr int kTowerRows = 5;
constexpr float kMaxFrameTime = 1.0f / 30.0f;
constexpr float kStepDt = 1.0f / 60.0f;

struct TowerBlock {
    b2Body* body;
    bool knocked;
};

class GameContactListener final : public b2ContactListener {
public:
    GameContactListener(b2Body* removalZone, std::vector<TowerBlock>& towerBlocks,
                        const std::vector<b2Body*>& cannonballs, int& knockedCount)
        : removalZone_(removalZone),
          towerBlocks_(towerBlocks),
          cannonballs_(cannonballs),
          knockedCount_(knockedCount) {}

    void BeginContact(b2Contact* contact) override {
        b2Body* bodyA = contact->GetFixtureA()->GetBody();
        b2Body* bodyB = contact->GetFixtureB()->GetBody();
        if (bodyA == removalZone_) {
            queueRemoval(bodyB);
            return;
        }
        if (bodyB == removalZone_) {
            queueRemoval(bodyA);
            return;
        }
        recordContact(contact);
    }

    void PreSolve(b2Contact* contact, const b2Manifold*) override {
        b2Body* bodyA = contact->GetFixtureA()->GetBody();
        b2Body* bodyB = contact->GetFixtureB()->GetBody();
        if (findBlock(bodyA) == nullptr || findBlock(bodyB) == nullptr) {
            return;
        }

        // Existing block contacts also need to report a hit when a block is driven into another.
        b2WorldManifold worldManifold;
        contact->GetWorldManifold(&worldManifold);
        const b2Manifold* manifold = contact->GetManifold();
        for (int i = 0; i < manifold->pointCount; ++i) {
            const b2Vec2 velocityA =
                bodyA->GetLinearVelocityFromWorldPoint(worldManifold.points[i]);
            const b2Vec2 velocityB =
                bodyB->GetLinearVelocityFromWorldPoint(worldManifold.points[i]);
            if (b2Dot(velocityB - velocityA, worldManifold.normal) < -b2_epsilon) {
                recordContact(contact);
                return;
            }
        }
    }

    bool processPendingContacts() {
        bool scoreChanged = false;
        bool progress;
        do {
            progress = false;
            for (const auto& [bodyA, bodyB] : pendingContacts_) {
                TowerBlock* blockA = findBlock(bodyA);
                TowerBlock* blockB = findBlock(bodyB);

                if (blockA != nullptr && isCannonball(bodyB)) {
                    scoreChanged |= markKnocked(*blockA, progress);
                }
                if (blockB != nullptr && isCannonball(bodyA)) {
                    scoreChanged |= markKnocked(*blockB, progress);
                }
                if (blockA != nullptr && blockB != nullptr) {
                    if (blockA->knocked) {
                        scoreChanged |= markKnocked(*blockB, progress);
                    }
                    if (blockB->knocked) {
                        scoreChanged |= markKnocked(*blockA, progress);
                    }
                }
            }
        } while (progress);

        pendingContacts_.clear();
        return scoreChanged;
    }

    std::vector<b2Body*> takePendingRemovals() {
        std::vector<b2Body*> removals;
        removals.swap(pendingRemovals_);
        return removals;
    }

    void clearPendingEvents() {
        pendingContacts_.clear();
        pendingRemovals_.clear();
    }

private:
    void queueRemoval(b2Body* body) {
        if (body->GetType() != b2_staticBody &&
            std::find(pendingRemovals_.begin(), pendingRemovals_.end(), body) ==
                pendingRemovals_.end()) {
            pendingRemovals_.push_back(body);
        }
    }

    void recordContact(b2Contact* contact) {
        pendingContacts_.emplace_back(contact->GetFixtureA()->GetBody(),
                                      contact->GetFixtureB()->GetBody());
    }

    TowerBlock* findBlock(b2Body* body) {
        for (TowerBlock& block : towerBlocks_) {
            if (block.body == body) {
                return &block;
            }
        }
        return nullptr;
    }

    bool isCannonball(const b2Body* body) const {
        return std::find(cannonballs_.begin(), cannonballs_.end(), body) != cannonballs_.end();
    }

    bool markKnocked(TowerBlock& block, bool& progress) {
        if (block.knocked) {
            return false;
        }
        block.knocked = true;
        ++knockedCount_;
        progress = true;
        return true;
    }

    b2Body* removalZone_;
    std::vector<TowerBlock>& towerBlocks_;
    const std::vector<b2Body*>& cannonballs_;
    int& knockedCount_;
    std::vector<std::pair<b2Body*, b2Body*>> pendingContacts_;
    std::vector<b2Body*> pendingRemovals_;
};

enum class GameState {
    playing,
    win,
    lose,
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

b2Body* createOutOfBoundsZone(b2World& world) {
    b2BodyDef bodyDef;
    bodyDef.type = b2_staticBody;
    b2Body* zone = world.CreateBody(&bodyDef);

    const float halfThickness = kOutOfBoundsZoneThickness / 2.0f;
    const float halfWorldWidth = kWorldWidth / 2.0f;
    const float halfWorldHeight = kWorldHeight / 2.0f;
    const auto addSensorBox = [&](float centerX, float centerY, float halfWidth,
                                  float halfHeight) {
        b2PolygonShape shape;
        shape.SetAsBox(halfWidth, halfHeight, {centerX, centerY}, 0.0f);

        b2FixtureDef fixtureDef;
        fixtureDef.shape = &shape;
        fixtureDef.isSensor = true;
        zone->CreateFixture(&fixtureDef);
    };

    addSensorBox(-halfThickness, halfWorldHeight, halfThickness,
                 halfWorldHeight + kOutOfBoundsZoneThickness);
    addSensorBox(kWorldWidth + halfThickness, halfWorldHeight, halfThickness,
                 halfWorldHeight + kOutOfBoundsZoneThickness);
    addSensorBox(halfWorldWidth, -halfThickness,
                 halfWorldWidth + kOutOfBoundsZoneThickness, halfThickness);
    addSensorBox(halfWorldWidth, kWorldHeight + halfThickness,
                 halfWorldWidth + kOutOfBoundsZoneThickness, halfThickness);
    return zone;
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

std::vector<TowerBlock> createTowerBlocks(b2World& world) {
    std::vector<TowerBlock> blocks;
    const float groundTopY = kGroundY + kGroundHalfHeight;
    for (int row = 0; row < kTowerRows; ++row) {
        for (int column = 0; column < kTowerColumns; ++column) {
            const float x = kTowerLeftX + column * 2.0f * kTowerHalfSize;
            const float y = groundTopY + kTowerHalfSize + row * 2.0f * kTowerHalfSize;
            b2Body* body = createTowerBlock(world, x, y);
            blocks.push_back({body, false});
        }
    }
    return blocks;
}

b2Body* createCannonball(b2World& world, const b2Vec2& position) {
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.position = position;
    bodyDef.bullet = true;
    bodyDef.angularDamping = 0.5f;
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

void drawArcPreview(float speed, float angle, float gravityY) {
    const b2Vec2 start(kPivotX + std::cos(angle) * kBarrelLength,
                       kPivotY + std::sin(angle) * kBarrelLength);
    const float velocityX = speed * std::cos(angle);
    const float velocityY = speed * std::sin(angle);
    const float groundContactY = kGroundY + kGroundHalfHeight + kCannonballRadius;

    for (int i = 1; i <= kMaxArcDots; ++i) {
        const float time = i * kArcTimeStep;
        const b2Vec2 position(start.x + velocityX * time,
                              start.y + velocityY * time +
                                  0.5f * gravityY * time * time);
        if (position.x + kCannonballRadius >= kWorldWidth ||
            position.y <= groundContactY) {
            break;
        }

        gfx::drawCircle(gfx::metersToPixels(position.x, kPixelsPerMeter),
                        gfx::metersToPixelsY(position.y, kWindowHeight, kPixelsPerMeter),
                        gfx::metersToPixels(kArcDotRadius, kPixelsPerMeter), 1.0f, 1.0f,
                        1.0f);
    }
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

void drawAmmo(int shotsLeft) {
    for (int i = 0; i < shotsLeft; ++i) {
        gfx::drawBox(kAmmoSquareStartX + i * kAmmoSquareSpacing, kAmmoSquareY,
                     kAmmoSquareSize, kAmmoSquareSize, 0.0f, 0.9f, 0.1f, 0.1f);
    }
}

bool isSettled(const std::vector<TowerBlock>& towerBlocks,
               const std::vector<b2Body*>& cannonballs) {
    for (const TowerBlock& block : towerBlocks) {
        if (block.body->IsAwake()) {
            return false;
        }
    }
    for (const b2Body* ball : cannonballs) {
        if (ball->IsAwake()) {
            return false;
        }
    }
    return true;
}

void updateWindowTitle(const gfx::Window& window, int knockedCount, int shotsLeft,
                       GameState gameState) {
    std::string title = "knocked: " + std::to_string(knockedCount) + "/" +
                        std::to_string(kTowerColumns * kTowerRows) + " | shots left: " +
                        std::to_string(shotsLeft);
    if (gameState == GameState::win) {
        title += " WIN";
    } else if (gameState == GameState::lose) {
        title += " LOSE";
    }
    window.setTitle(title);
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
        b2Body* removalZone = createOutOfBoundsZone(world);

        std::vector<TowerBlock> towerBlocks = createTowerBlocks(world);
        std::vector<b2Body*> cannonballs;

        int knockedCount = 0;
        GameContactListener contactListener(removalZone, towerBlocks, cannonballs, knockedCount);
        world.SetContactListener(&contactListener);
        int shotsLeft = kMaxShots;
        GameState gameState = GameState::playing;
        updateWindowTitle(window, knockedCount, shotsLeft, gameState);

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        double previousTime = glfwGetTime();
        float power = kMinPower;
        float accumulator = 0.0f;
        bool wasMouseDown = false;
        bool rWasDown = false;

        while (!window.shouldClose()) {
            gfx::processInput(window);
            const bool rDown = window.isKeyPressed(GLFW_KEY_R);
            const bool resetPressed = rDown && !rWasDown;
            rWasDown = rDown;
            if (resetPressed) {
                contactListener.clearPendingEvents();
                for (const TowerBlock& block : towerBlocks) {
                    world.DestroyBody(block.body);
                }
                for (b2Body* ball : cannonballs) {
                    world.DestroyBody(ball);
                }
                towerBlocks = createTowerBlocks(world);
                cannonballs.clear();
                knockedCount = 0;
                shotsLeft = kMaxShots;
                gameState = GameState::playing;
                power = kMinPower;
                accumulator = 0.0f;
                wasMouseDown = false;
                updateWindowTitle(window, knockedCount, shotsLeft, gameState);
            }

            const bool mouseDown = window.isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT);
            const bool mouseReleased = wasMouseDown && !mouseDown;
            const GameState previousGameState = gameState;
            const bool canCharge =
                mouseDown && shotsLeft > 0 && gameState == GameState::playing;

            const double currentTime = glfwGetTime();
            const float elapsedTime = static_cast<float>(currentTime - previousTime);
            previousTime = currentTime;
            const float frameTime = std::min(elapsedTime, kMaxFrameTime);
            float launchPower = kMinPower;
            if (canCharge) {
                power = std::min(kMaxPower, power + kChargeRate * elapsedTime);
            } else {
                if (mouseReleased && shotsLeft > 0 && gameState == GameState::playing) {
                    launchPower = power;
                }
                power = kMinPower;
            }

            bool knockedCountChanged = false;
            accumulator += frameTime;
            while (accumulator >= kStepDt) {
                world.Step(kStepDt, 8, 3);
                knockedCountChanged =
                    contactListener.processPendingContacts() || knockedCountChanged;
                for (b2Body* body : contactListener.takePendingRemovals()) {
                    world.DestroyBody(body);
                    towerBlocks.erase(
                        std::remove_if(towerBlocks.begin(), towerBlocks.end(),
                                       [body](const TowerBlock& block) {
                                           return block.body == body;
                                       }),
                        towerBlocks.end());
                    cannonballs.erase(std::remove(cannonballs.begin(), cannonballs.end(), body),
                                      cannonballs.end());
                }
                accumulator -= kStepDt;
            }

            if (knockedCount == kTowerColumns * kTowerRows) {
                gameState = GameState::win;
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

            bool shotFired = false;
            if (mouseReleased && shotsLeft > 0 && gameState == GameState::playing) {
                const b2Vec2 muzzlePosition(
                    kPivotX + std::cos(aimAngle) * kBarrelLength,
                    kPivotY + std::sin(aimAngle) * kBarrelLength);
                b2Body* ball = createCannonball(world, muzzlePosition);
                const float impulseMagnitude = ball->GetMass() * launchPower;
                ball->ApplyLinearImpulseToCenter(
                    {impulseMagnitude * std::cos(aimAngle),
                     impulseMagnitude * std::sin(aimAngle)},
                    true);
                cannonballs.push_back(ball);
                --shotsLeft;
                shotFired = true;
            }
            wasMouseDown = mouseDown;
            if (gameState == GameState::playing && shotsLeft == 0 &&
                isSettled(towerBlocks, cannonballs)) {
                gameState = GameState::lose;
            }
            if (knockedCountChanged || shotFired || gameState != previousGameState) {
                updateWindowTitle(window, knockedCount, shotsLeft, gameState);
            }

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
            drawArcPreview(power, aimAngle, world.GetGravity().y);
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
            drawAmmo(shotsLeft);
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
