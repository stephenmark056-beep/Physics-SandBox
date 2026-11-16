# Physics Sandbox

A C++20 and OpenGL starter for physics projects on Windows. It opens an 800x600
window with a teal background, a gray ground bar, and a gray cannon base at the
left. The ground and base are static Box2D bodies; their fixtures define the
geometry used for rendering. Box2D v2.4.2 is vendored in the project.

GLFW, GLAD and Box2D are vendored, so nothing needs to be installed beyond the
compiler.

## Prerequisites

- **MinGW-w64 via MSYS2**. Install from [msys2.org](https://www.msys2.org/),
  then install the `mingw-w64-x86_64-gcc` toolchain from the MSYS2 shell.
- **VS Code**.
- The MinGW `bin` folder (e.g. `C:\msys64\mingw64\bin`) on your system `PATH`.
  The build tasks call `g++` and `ar` by name, and the compiler's own child
  processes find their DLLs through the system `PATH`, so without this entry
  the build can fail silently.

## Build and run

1. Open the folder in VS Code.
2. Run the `Build Main` task (`Ctrl+Shift+B`).
3. Run `main.exe` from the repo folder. Press `Esc` to close it.

## Box2D library

`lib/libbox2d.a` is not committed, because a static C++ library built with one
GCC version can fail to link with another. Each machine builds its own. To
build it after switching compilers or editing `src/box2d/`, run the
`Build Box2D library` task. The empty window does not link Box2D; the library
is kept for future physics work.

## Project structure

```
.vscode/tasks.json        build tasks
glfw3.dll                 GLFW runtime, must sit next to main.exe
lib/libglfw3dll.a         GLFW import library
src/
  main.cpp                Box2D ground and cannon base, background and loop
  draw.h/.cpp             reusable box and line renderer
  glad.c                  vendored GLAD loader
  core/                   GLFW context, window and keyboard input (gfx namespace)
  box2d/                  vendored Box2D v2.4.2 sources and its LICENSE
include/
  box2d/                  Box2D headers
  glad/, GLFW/, KHR/      vendored GLAD and GLFW headers
scripts/
  build_box2d.bat         compiles src/box2d into lib/libbox2d.a
```

## Troubleshooting

- **`cannot open output file main.exe: Permission denied`.** The program is
  still running. Close its window and build again.
- **`g++` is not recognized.** The MinGW `bin` folder is not on your `PATH`.
  Add it, then restart VS Code.
- **Program compiles but won't run.** `glfw3.dll` must sit next to `main.exe`.
