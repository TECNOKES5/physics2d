# physics2d

A 2D rigid body physics engine in C++ with OpenGL rendering.

## Quick start

```bash
# Install GLFW (pick your display server)
sudo pacman -S glfw-x11       # X11
# sudo pacman -S glfw-wayland # Wayland

chmod +x build.sh
./build.sh
```

## Controls

| Key / Input   | Action                  |
|---------------|-------------------------|
| Left click    | Spawn body at cursor    |
| `C`           | Switch to circle mode   |
| `B`           | Switch to box mode      |
| `R`           | Reset scene             |
| `G`           | Flip gravity            |
| `Space`       | Pause / unpause         |
| `Escape`      | Quit                    |

## Architecture

```
src/
 ├── physics.h   – Vec2, Shape, Body, World, collision detection, impulse solver
 ├── renderer.h  – OpenGL immediate-mode renderer (circles, boxes, grid)
 └── main.cpp    – GLFW window, input callbacks, scene setup
```

### physics.h

- **Vec2** – small vector math struct (dot, cross, norm, rotate)
- **Shape** – circle (radius) or box (half-extents)
- **Body** – position, velocity, angle, angular velocity, mass, inertia
- **Manifold** – collision contact info (normal, depth, contact points)
- Collision detection: circle-circle, box-box (SAT), circle-box
- Impulse resolution with friction + positional correction (anti-sinking)
- **World** – fixed-timestep integration loop (120 Hz substep), multi-iteration solver

### renderer.h

- Legacy OpenGL immediate mode (no shader boilerplate)
- Orthographic camera with zoom + pan
- `screenToWorld()` for click-to-spawn
