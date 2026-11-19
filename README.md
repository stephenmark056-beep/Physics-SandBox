# Physics Sandbox

A C++20, OpenGL, and Box2D physics sandbox for Windows. It opens an 800x600
window with a cannon, ground, and a tower of blocks. Box2D v2.4.2 handles
projectile physics, collision filtering, and contact-based gameplay.

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

## Controls

- Hold and release the left mouse button to charge and fire.
- Press `1` to select a normal ball or `2` to select a ghost ball. Ghost balls
  pass through blocks but collide with the ground.
- Press `3` to select a bomb. It explodes on its first solid contact and pushes
  nearby blocks away.
- Press `R` to reset the game.

## Box2D library

`lib/libbox2d.a` is not committed, because a static C++ library built with one
GCC version can fail to link with another. Each machine builds its own. To
build it after switching compilers or editing `src/box2d/`, run the
`Build Box2D library` task. The `Build Main` task builds the library if it is
missing and links it into the game.

## Project structure

```
.vscode/tasks.json        build tasks
glfw3.dll                 GLFW runtime, must sit next to main.exe
lib/libglfw3dll.a         GLFW import library
src/
  main.cpp                cannon, tower, ammo, Box2D contacts and game loop
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
