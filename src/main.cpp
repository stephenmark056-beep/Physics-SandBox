#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <cstdlib>
#include <exception>
#include <iostream>

#include "core/glfw_context.h"
#include "core/input.h"
#include "core/window.h"

namespace {

constexpr int kWindowWidth = 800;
constexpr int kWindowHeight = 600;

}  // namespace

int main() {
    try {
        gfx::GlfwContext glfw;
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
        gfx::Window window({.width = kWindowWidth, .height = kWindowHeight,
                            .title = "Physics Sandbox"});

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);

        while (!window.shouldClose()) {
            gfx::processInput(window);
            glClear(GL_COLOR_BUFFER_BIT);
            window.swapBuffers();
            window.pollEvents();
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
