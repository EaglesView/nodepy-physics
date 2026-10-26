# nodepy-physics 🪢🧱

Rigid-body 2D physics for the [NodePy](https://github.com/OneTrain2005/NodePy) engine, powered by [pymunk](https://www.pymunk.org/) (Chipmunk2D).

**NOTE**: Nodepy is still a WIP. This is for an early development version of NodePy. [See this PR](https://github.com/OneTrain2005/NodePy/pull/4) for more info.

Gravity, stacking, friction, bounce, rotation, joints and collision signals, exposed as Godot-style nodes you drop into your scene tree. No pymunk knowledge required, but the underlying objects are one property away when you need them.

> **Version:** 0.1.0
> **Requires:** NodePy 1.2+, pymunk 7, Python 3.10+

---

## Table of contents

- [nodepy-physics 🪢🧱](#nodepy-physics-)
  - [Table of contents](#table-of-contents)
  - [Getting started](#getting-started)
  - [Project structure](#project-structure)
  - [Quick example](#quick-example)
  - [How it plugs into NodePy](#how-it-plugs-into-nodepy)
  - [Using the plugin](#using-the-plugin)
    - [PhysicsServer](#physicsserver)
    - [Bodies](#bodies)
    - [Shapes](#shapes)
    - [Joints](#joints)
    - [Collision signals](#collision-signals)
    - [Drawing and debug view](#drawing-and-debug-view)
    - [Dropping down to pymunk](#dropping-down-to-pymunk)
  - [Units and conventions](#units-and-conventions)
  - [Known limitations](#known-limitations)
  - [Contributing](#contributing)
  - [License](#license)

---

## Getting started

You need **Python 3.10+** with tkinter (see the [NodePy README](https://github.com/OneTrain2005/NodePy#getting-started) if it is missing).

Install the engine first, from GitHub:

```bash
pip install git+https://github.com/OneTrain2005/NodePy.git
```

> **Why not `pip install NodePy`?** The `NodePy` name on PyPI belongs to an unrelated numerical-ODE package, which also imports as `nodepy`. Install the engine from GitHub, and don't install both in the same environment.

Then install the plugin (this pulls in pymunk):

```bash
pip install git+https://github.com/OneTrain2005/nodepy-physics.git
```

Or clone it and run the demo:

```bash
git clone https://github.com/OneTrain2005/nodepy-physics.git
cd nodepy-physics
pip install -e .
python3 demos/knockdown/main.py
```

**Knockdown** is a small slingshot game: drag back from the slingshot and release to fire, knock the green targets off their towers, or swing the wrecking ball into them. Press **D** to show every collision shape and **R** to restart.

---

## Project structure

```
.
├── nodepy_physics
│   ├── __init__.py         # Public API: everything exported here is stable
│   ├── PhysicsServer.py    # Plugin that owns the pymunk Space and steps it
│   ├── PhysicsBody2D.py    # StaticBody2D, KinematicBody2D, RigidBody2D
│   ├── Shape2D.py          # Box, circle, polygon and segment collision shapes
│   └── Joint2D.py          # Pin, rope and damped spring joints
├── demos
│   └── knockdown/          # Slingshot game: stacking, joints, impact signals
├── pyproject.toml
├── LICENSE                 # MIT
└── README.md
```

---

## Quick example

A ball dropped onto a slope:

```python
from nodepy import GameLoop, Node, Vector2d
from nodepy_physics import (
    CircleShape2D, PhysicsServer, RigidBody2D, SegmentShape2D, StaticBody2D,
)

world = Node("World")

ground = StaticBody2D("Ground", parent=world)
SegmentShape2D("shape", Vector2d(50, 400), Vector2d(750, 550),
               thickness=6, color="#8d99ae", parent=ground)

ball = RigidBody2D("Ball", relative_pos=Vector2d(150, 100), parent=world)
CircleShape2D("shape", 20, elasticity=0.5, color="#e9ecef",
              outline="#adb5bd", parent=ball)

loop = GameLoop(width=800, height=600, title="Physics")
loop.add_plugin(PhysicsServer(gravity=Vector2d(0, 900)))   # before set_scene
loop.set_scene(world)
loop.run()
```

The pattern is always the same: a **body** node holding one or more **shape** children, placed under an untransformed "world" node, with a `PhysicsServer` installed on the loop.

---

## How it plugs into NodePy

`PhysicsServer` is a NodePy [`Plugin`](https://github.com/OneTrain2005/NodePy#plugins). Each fixed physics tick (60 Hz) runs in this order:

```
1. Every node's _physics_process(delta)      ← apply forces, impulses, velocities here
2. PhysicsServer.on_post_physics(delta)
   - add bodies / shapes / joints that entered the tree this tick
   - space.step(delta)
   - copy each awake body's position and rotation back into its node
   - emit body_entered / body_exited signals
3. NodePy's own CollisionShape detection
```

This mirrors Godot: your code decides what to do, the simulation resolves it, and the scene tree reflects the result before `_process` and rendering run.

Registration is automatic. Bodies, shapes and joints add themselves to the space when they enter the tree and remove themselves when they leave it (`queue_free()`, `remove_child()`), including re-adding when a node is moved back in. Additions are queued until the next step, so it is safe to spawn or free bodies from inside signal handlers.

---

## Using the plugin

Import everything from the package root:

```python
from nodepy_physics import (
    PhysicsServer,
    StaticBody2D, KinematicBody2D, RigidBody2D,
    BoxShape2D, CircleShape2D, PolygonShape2D, SegmentShape2D,
    PinJoint2D, RopeJoint2D, DampedSpringJoint2D,
)
```

### PhysicsServer

```python
PhysicsServer(gravity=Vector2d(0, 900), iterations=20, sleep_time=0.5)
```

| Parameter | Default | Meaning |
|---|---|---|
| `gravity` | `Vector2d(0, 900)` | World gravity in px/s². Y points down. |
| `iterations` | `20` | Solver iterations per step. Raise it if tall stacks feel soft. |
| `sleep_time` | `0.5` | Seconds a body must stay still before it sleeps. Sleeping bodies cost nothing until something touches them. |

Install it **before** `loop.set_scene()`, so bodies can register as they enter the tree. Only one server can be installed at a time. Anywhere in your code you can reach it with `PhysicsServer.get()` (or `loop.get_plugin("physics")`), for example to change `PhysicsServer.get().space.gravity` at runtime.

### Bodies

| Node | Moved by | Use for |
|---|---|---|
| `StaticBody2D` | Nothing (unless you teleport it) | Ground, walls, platforms |
| `KinematicBody2D` | Your code, via `linear_velocity` / `angular_velocity` | Moving platforms, doors, elevators. Pushes rigid bodies, ignores forces. |
| `RigidBody2D` | The simulation | Crates, balls, ragdolls, projectiles |

All bodies take `name`, `relative_pos`, `rotation` (degrees), `contact_monitor` and `parent`.

**Setting `relative_pos` or `rotation` teleports the body.** Use it to spawn and reset things; for motion, use velocities, forces and impulses so collisions resolve properly.

`KinematicBody2D` and `RigidBody2D` share:

```python
body.linear_velocity = Vector2d(200, 0)   # px/s
body.angular_velocity = 90                # degrees/s, clockwise
```

`RigidBody2D` adds:

```python
body.apply_impulse(Vector2d(0, -400))                    # instant kick, world space
body.apply_impulse(Vector2d(300, 0), Vector2d(0, -10))   # ...at an offset from the centre (local space)
body.apply_force(Vector2d(0, -2000))                     # continuous push; call every tick from _physics_process
body.mass                                                # sum of shape area × density
body.sleeping                                            # True while at rest
body.wake_up()
```

A body subclass can override `_ready`, `_physics_process` and friends without calling `super()`; the shapes take care of registering their body.

```python
class Player(RigidBody2D):
    def __init__(self):
        super().__init__("Player", relative_pos=Vector2d(100, 100))
        BoxShape2D("shape", 32, 48, friction=1.0, color="#4cc9f0", parent=self)

    def _physics_process(self, delta):
        if Input.is_action_just_pressed("jump"):
            self.apply_impulse(Vector2d(0, -350 * self.mass))
```

### Shapes

Shapes are the collision geometry of their **parent body**, and must be its children. One body can carry several shapes: each shape's own `relative_pos` and `rotation` offset it from the body.

| Node | Geometry |
|---|---|
| `BoxShape2D(name, width, height)` | Rectangle centred on the node |
| `CircleShape2D(name, radius)` | Circle centred on the node, drawn with a spoke so rolling is visible |
| `PolygonShape2D(name, points)` | Convex polygon from a list of `Vector2d`. Concave lists are wrapped in their convex hull. |
| `SegmentShape2D(name, a, b, thickness=2)` | Thick line from `a` to `b`. Ideal for ground, slopes and walls. |

Every shape also accepts:

| Parameter | Default | Meaning |
|---|---|---|
| `density` | `1.0` | Mass per px² of area. A body's mass is the sum over its shapes. |
| `friction` | `0.7` | 0 = ice, ~1 = rubber |
| `elasticity` | `0.1` | Bounciness: 0 = none, 1 = perfect bounce |
| `color` | `None` | Fill colour, or `None` to stay invisible |
| `outline` | `""` | Outline colour when drawn |
| `relative_pos` | origin | Offset from the body |

A compound body, a hammer from two boxes:

```python
hammer = RigidBody2D("Hammer", relative_pos=Vector2d(400, 100), parent=world)
BoxShape2D("handle", 10, 80, density=0.5, color="#b5835a", parent=hammer)
BoxShape2D("head", 50, 20, density=4, color="#5c6378",
           relative_pos=Vector2d(0, -40), parent=hammer)
```

### Joints

Joints connect two bodies. Pass `None` as `body_b` to pin `body_a` to the world itself. Joints are nodes too: put them anywhere in the tree (usually next to the bodies) and they appear and disappear with it.

| Node | Behaviour |
|---|---|
| `PinJoint2D(name, body_a, body_b)` | Hinge. Both bodies rotate around one shared point: the joint node's own `relative_pos` (world space). |
| `RopeJoint2D(name, body_a, body_b, anchor_a, anchor_b, length)` | Keeps the anchors at most `length` apart, slack allowed. |
| `DampedSpringJoint2D(name, body_a, body_b, anchor_a, anchor_b, rest_length, stiffness=50, damping=1)` | A spring between the anchors. |

Anchors are local to each body, or world space when that body is `None`. Leave `length` / `rest_length` out to use the current distance between the anchors. Give a joint a `color` (and optional `width`) to draw it as a line.

A wrecking ball hanging from a fixed point, taken from the demo:

```python
anchor = Vector2d(520, 40)
ball = RigidBody2D("WreckingBall", relative_pos=anchor + Vector2d(0, 300), parent=world)
CircleShape2D("shape", 26, density=4, color="#495057", parent=ball)
RopeJoint2D("Rope", ball, None, anchor_b=anchor, length=300,
            color="#8d99ae", parent=world)
```

A door on a hinge:

```python
door = RigidBody2D("Door", relative_pos=Vector2d(300, 200), parent=world)
BoxShape2D("shape", 10, 100, parent=door)
hinge = PinJoint2D("Hinge", door, None, parent=world)
hinge.relative_pos = Vector2d(300, 150)   # set before the joint enters the tree
```

### Collision signals

Collision signals cost a Python callback per contact, so they are **opt-in** per body with `contact_monitor=True`:

```python
class Target(RigidBody2D):
    def __init__(self, pos):
        super().__init__("Target", relative_pos=pos, contact_monitor=True)
        CircleShape2D("shape", 16, color="#7bd389", parent=self)
        self.body_entered.connect(self._on_body_entered)

    def _on_body_entered(self, other, impact_speed):
        if impact_speed > 220:      # px/s along the contact normal
            self.queue_free()
```

| Signal | Arguments | When |
|---|---|---|
| `body_entered` | `other`, `impact_speed` | Another body started touching this one |
| `body_exited` | `other` | It stopped touching |

`impact_speed` is the relative speed of the two bodies along the contact normal, handy for "break on a hard hit" logic. Signals are emitted after the step, so freeing or spawning bodies in a handler is safe.

### Drawing and debug view

Physics shapes and joints draw themselves on the NodePy canvas when given a `color`, which is enough for prototypes and simple flat art. For real art, leave shapes invisible and add a core `ColorRect2D` or `Sprite2D` as a sibling child of the body. It follows the body's position **and rotation** automatically, because bodies are ordinary nodes:

```python
crate = RigidBody2D("Crate", relative_pos=Vector2d(400, 100), parent=world)
BoxShape2D("shape", 40, 40, parent=crate)                         # invisible
Sprite2D("visual", texture=crate_texture, parent=crate)           # what you see
```

Toggle an outline of every collision shape in the scene, great for checking that art and hitboxes line up:

```python
Shape2D.debug_draw = True
Shape2D.debug_color = "#4cc9f0"   # optional
```

### Dropping down to pymunk

The node API covers the common cases. For everything else (collision filtering, motors, gear joints, raycasts, custom handlers), the underlying pymunk objects are exposed:

```python
PhysicsServer.get().space        # pymunk.Space
body.pymunk_body                 # pymunk.Body
shape.pymunk_shape               # pymunk.Shape (once the shape is in the tree)
joint.pymunk_constraint          # pymunk.Constraint (once the joint is in the tree)
```

```python
import pymunk

# Raycast from the player to the mouse
mouse = Input.mouse_position()
hit = PhysicsServer.get().space.segment_query_first(
    (player.relative_pos.x, player.relative_pos.y),
    (mouse.x, mouse.y), 1, pymunk.ShapeFilter())
```

---

## Units and conventions

| Quantity | Unit |
|---|---|
| Position, size | Pixels, Y pointing down (same as NodePy) |
| Rotation, angular velocity | Degrees, degrees per second, clockwise (same as NodePy) |
| Velocity | px/s |
| Gravity, acceleration | px/s² |
| Density | Mass per px² |

pymunk itself works in radians; the conversion happens at the node boundary. Values read straight from `pymunk_body` are in radians.

---

## Known limitations

| Limitation | Impact | Workaround |
|---|---|---|
| **Bodies use world coordinates** | A body's `relative_pos` and `rotation` are fed straight to pymunk. Under a moved, rotated or scaled parent, physics and drawing disagree. | Put bodies directly under an untransformed "world" node. Use `Camera2D` to scroll, not a moving parent. |
| **Shape geometry is fixed once created** | Changing `width`, `radius`, `density`, `friction`... after a shape enters the tree has no effect on the simulation. | Set values before adding the shape, or edit `shape.pymunk_shape` directly. |
| **One PhysicsServer per process** | The active server is a class-level singleton. | Install a single server; remove it (or shut the loop down) before installing another. |
| **No collision layers in the node API** | Every shape collides with every other shape. | Set `shape.pymunk_shape.filter = pymunk.ShapeFilter(...)` once the shape is in the tree. |
| **Separate from core `CollisionShape`** | NodePy's AABB `CollisionShape` and these physics shapes don't interact. | Use physics shapes plus `contact_monitor` for physics objects; core `CollisionShape` stays fine for simple triggers. |
| **2D only** | — | — |

---

## Contributing

Issues and pull requests are welcome. Follow the [NodePy plugin conventions](https://github.com/OneTrain2005/NodePy#writing-a-plugin-package): import only what `nodepy/__init__.py` exports, and keep anything that isn't part of this package's public API prefixed with `_`.

---

## License

MIT — see [LICENSE](LICENSE).
