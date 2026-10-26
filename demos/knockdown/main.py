"""
Knockdown — demo for nodepy-physics.  Built with NodePy 1.2.

Drag back from the slingshot and release to fire.  Knock the green targets
off their towers, or swing the wrecking ball into them.

What it shows
-------------
- RigidBody2D / StaticBody2D with box, circle and segment shapes
- Core ColorRect2D nodes riding on rotating physics bodies
- RopeJoint2D holding a wrecking ball
- body_entered(other, impact_speed) signals deciding when targets break
- Shape2D.debug_draw to reveal every collision shape (press D)
"""

import math
import sys
from pathlib import Path

# Allow running from a clone without installing nodepy-physics
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from nodepy import ColorRect2D, GameLoop, Input, Node, Signal, Vector2d
from nodepy_physics import (
    BoxShape2D, CircleShape2D, PhysicsServer, RigidBody2D, RopeJoint2D,
    Shape2D, StaticBody2D,
)

WIDTH, HEIGHT = 1000, 600
GRAVITY = 900.0
GROUND_Y = 560
SLING = Vector2d(150, 430)
MAX_PULL = 110.0
LAUNCH_POWER = 8.0
SHOTS = 4
BREAK_SPEED = 220.0

WOOD, WOOD_EDGE = "#b5835a", "#7a5232"
STONE, STONE_EDGE = "#5c6378", "#3e4354"
TARGET, TARGET_EDGE = "#7bd389", "#3f8f4c"

Input.add_action("restart", ["r", "R"])
Input.add_action("debug_shapes", ["d", "D"])


# ─────────────────────────────────────────────────────────────────────────────
# Bodies
# ─────────────────────────────────────────────────────────────────────────────


class Block(RigidBody2D):
    """Wooden block.  The visual is a plain core ColorRect2D child."""

    def __init__(self, pos: Vector2d, width: float, height: float, level: "Level"):
        super().__init__("Block", relative_pos=pos)
        self.level = level
        self._fallen = False
        BoxShape2D("shape", width, height, friction=0.8, parent=self)
        ColorRect2D("visual", width=width, height=height,
                    color=WOOD, outline=WOOD_EDGE, parent=self)

    def _physics_process(self, delta: float) -> None:
        if not self._fallen and self.relative_pos.y > GROUND_Y - 40:
            self._fallen = True
            self.level.add_score(50)
        if self.relative_pos.y > HEIGHT + 200 or self.relative_pos.x > WIDTH + 200:
            self.queue_free()


class Target(RigidBody2D):
    """Breaks on a hard hit, or when it falls to the ground."""

    def __init__(self, pos: Vector2d, level: "Level"):
        super().__init__("Target", relative_pos=pos, contact_monitor=True)
        self.level = level
        CircleShape2D("shape", 16, density=0.5, friction=0.8,
                      color=TARGET, outline=TARGET_EDGE, parent=self)
        self.body_entered.connect(self._on_body_entered)

    def _on_body_entered(self, other, impact_speed: float) -> None:
        if impact_speed > BREAK_SPEED:
            self.destroy()

    def _physics_process(self, delta: float) -> None:
        if self.relative_pos.y > GROUND_Y - 40:
            self.destroy()

    def destroy(self) -> None:
        if not self._queued_for_free:
            self.queue_free()
            self.level.target_destroyed()


class Ball(RigidBody2D):
    def __init__(self, pos: Vector2d, velocity: Vector2d):
        super().__init__("Ball", relative_pos=pos)
        CircleShape2D("shape", 14, density=6, elasticity=0.3, friction=0.6,
                      color="#e9ecef", outline="#adb5bd", parent=self)
        self.linear_velocity = velocity
        self.age = 0.0
        self.gone = False
        self.tree_exited.connect(self._on_gone)

    def _on_gone(self, node) -> None:
        self.gone = True

    def _physics_process(self, delta: float) -> None:
        self.age += delta
        p = self.relative_pos
        if p.x > WIDTH + 100 or p.x < -100 or p.y > HEIGHT + 100:
            self.queue_free()

    @property
    def settled(self) -> bool:
        return self.gone or self.sleeping or self.age > 8.0


# ─────────────────────────────────────────────────────────────────────────────
# Level
# ─────────────────────────────────────────────────────────────────────────────


class Level(Node):
    """Builds the scene and keeps the score.  Freed and rebuilt on restart."""

    def __init__(self):
        super().__init__("Level")
        self.score = 0
        self.shots_left = SHOTS
        self.targets_left = 0
        self.balls: list = []
        self.changed = Signal("changed")

        # Ground
        ground = StaticBody2D("Ground", relative_pos=Vector2d(WIDTH / 2, GROUND_Y + 40),
                              parent=self)
        BoxShape2D("shape", WIDTH + 600, 80, color="#2b2d42", outline="", parent=ground)

        # Pedestals
        self._pedestal(Vector2d(680, 515), 170, 90)    # top at y=470
        self._pedestal(Vector2d(880, 470), 170, 180)   # top at y=380

        self._tower(680, 470, floors=2)
        self._pyramid(880, 380)
        self._wrecking_ball(Vector2d(520, 40), length=300)

    def _pedestal(self, center: Vector2d, w: float, h: float) -> None:
        body = StaticBody2D("Pedestal", relative_pos=center, parent=self)
        BoxShape2D("shape", w, h, friction=0.9, color=STONE, outline=STONE_EDGE,
                   parent=body)

    def _tower(self, cx: float, base_y: float, floors: int) -> None:
        post_w, post_h, roof_h, span = 14, 70, 14, 120
        y = base_y
        for _ in range(floors):
            for dx in (-span / 2 + post_w / 2, span / 2 - post_w / 2):
                self.add_child(Block(Vector2d(cx + dx, y - post_h / 2), post_w, post_h, self))
            self._target(Vector2d(cx, y - 16))
            y -= post_h
            self.add_child(Block(Vector2d(cx, y - roof_h / 2), span, roof_h, self))
            y -= roof_h

    def _pyramid(self, cx: float, base_y: float) -> None:
        size = 36
        for row, count in enumerate((3, 2, 1)):
            y = base_y - size / 2 - row * size
            for i in range(count):
                x = cx + (i - (count - 1) / 2) * (size + 2)
                self.add_child(Block(Vector2d(x, y), size, size, self))
        self._target(Vector2d(cx, base_y - 3 * size - 16))

    def _target(self, pos: Vector2d) -> None:
        self.add_child(Target(pos, self))
        self.targets_left += 1

    def _wrecking_ball(self, anchor: Vector2d, length: float) -> None:
        ball = RigidBody2D("WreckingBall", relative_pos=anchor + Vector2d(0, length),
                           parent=self)
        CircleShape2D("shape", 26, density=4, friction=0.5,
                      color="#495057", outline="#212529", parent=ball)
        RopeJoint2D("Rope", ball, None, anchor_b=anchor, length=length,
                    color="#8d99ae", parent=self)
        pivot = ColorRect2D("RopeAnchor", width=10, height=10,
                            color="#8d99ae", outline="", parent=self)
        pivot.relative_pos = anchor

    # ── Game state ───────────────────────────────────────────────────────────

    def fire(self, pos: Vector2d, velocity: Vector2d) -> None:
        self.shots_left -= 1
        ball = Ball(pos, velocity)
        self.balls.append(ball)
        self.add_child(ball)
        self.changed.emit()

    def add_score(self, points: int) -> None:
        self.score += points
        self.changed.emit()

    def target_destroyed(self) -> None:
        self.targets_left -= 1
        self.add_score(500)

    @property
    def won(self) -> bool:
        return self.targets_left == 0

    @property
    def lost(self) -> bool:
        return (not self.won and self.shots_left == 0
                and all(b.settled for b in self.balls))


# ─────────────────────────────────────────────────────────────────────────────
# Slingshot  (input + aiming visuals, no physics of its own)
# ─────────────────────────────────────────────────────────────────────────────


class Slingshot(Node):
    def __init__(self, game: "Game"):
        super().__init__("Slingshot")
        self.game = game
        self.pull = None   # Vector2d offset from SLING while dragging

        # Two fork arms on a trunk, drawn with core ColorRect2D nodes
        for dx in (-14, 14):
            arm = ColorRect2D("Arm", width=8, height=40, color=WOOD,
                              outline=WOOD_EDGE, parent=self)
            arm.relative_pos = Vector2d(SLING.x + dx, SLING.y + 25)
        bar = ColorRect2D("Bar", width=36, height=8, color=WOOD,
                          outline=WOOD_EDGE, parent=self)
        bar.relative_pos = Vector2d(SLING.x, SLING.y + 45)
        trunk_h = GROUND_Y - (SLING.y + 45)
        trunk = ColorRect2D("Trunk", width=10, height=trunk_h, color=WOOD,
                            outline=WOOD_EDGE, parent=self)
        trunk.relative_pos = Vector2d(SLING.x, SLING.y + 45 + trunk_h / 2)

    def _process(self, delta: float) -> None:
        level = self.game.level
        mouse = Input.mouse_position()
        can_fire = level.shots_left > 0 and not level.won

        if self.pull is None:
            if can_fire and Input.mouse_just_pressed(1) and (mouse - SLING).length() < 70:
                self.pull = Vector2d()
            return

        offset = mouse - SLING
        if offset.length() > MAX_PULL:
            offset = offset.normalized() * MAX_PULL
        self.pull = offset

        if Input.mouse_just_released(1):
            if self.pull.length() > 10:
                level.fire(SLING + self.pull, self.pull * -LAUNCH_POWER)
            self.pull = None

    def _draw(self, canvas, cam) -> None:
        if self.pull is None:
            # Idle band across the fork
            self._canvas_ids.append(canvas.create_line(
                SLING.x - 14, SLING.y + 8, SLING.x + 14, SLING.y + 8,
                fill="#6c3b1e", width=3))
            return

        pos = SLING + self.pull
        for dx in (-14, 14):
            self._canvas_ids.append(canvas.create_line(
                SLING.x + dx, SLING.y + 8, pos.x, pos.y, fill="#6c3b1e", width=3))

        # Trajectory preview: same gravity the PhysicsServer uses
        v = self.pull * -LAUNCH_POWER
        for i in range(1, 22):
            t = i * 0.06
            x = pos.x + v.x * t
            y = pos.y + v.y * t + 0.5 * GRAVITY * t * t
            if y > GROUND_Y:
                break
            r = max(1.5, 4 - i * 0.12)
            self._canvas_ids.append(canvas.create_oval(
                x - r, y - r, x + r, y + r, fill="#adb5bd", outline=""))

        self._canvas_ids.append(canvas.create_oval(
            pos.x - 14, pos.y - 14, pos.x + 14, pos.y + 14,
            fill="#e9ecef", outline="#adb5bd"))


# ─────────────────────────────────────────────────────────────────────────────
# HUD
# ─────────────────────────────────────────────────────────────────────────────


class HUD(Node):
    def __init__(self, game: "Game"):
        super().__init__("HUD")
        self.game = game

    def _draw(self, canvas, cam) -> None:
        level = self.game.level
        items = self._canvas_ids
        items.append(canvas.create_text(
            16, 14, anchor="nw", text=f"Score  {level.score}",
            fill="white", font=("Helvetica", 16, "bold")))
        items.append(canvas.create_text(
            16, 40, anchor="nw",
            text=f"Shots  {'●' * level.shots_left}{'○' * (SHOTS - level.shots_left)}"
                 f"     Targets  {level.targets_left}",
            fill="#ced4da", font=("Helvetica", 11)))
        items.append(canvas.create_text(
            16, HEIGHT - 14, anchor="sw",
            text="Drag back from the slingshot and release to fire  ·  "
                 "D  show collision shapes  ·  R  restart",
            fill="#8d99ae", font=("Helvetica", 10)))

        message = None
        if level.won:
            message = "Knocked down!  Press R to play again"
        elif level.lost:
            message = "Out of shots.  Press R to retry"
        if message:
            items.append(canvas.create_text(
                WIDTH / 2, 150, text=message, fill="white",
                font=("Helvetica", 22, "bold")))


# ─────────────────────────────────────────────────────────────────────────────
# Game
# ─────────────────────────────────────────────────────────────────────────────


class Game(Node):
    def __init__(self):
        super().__init__("Game")
        # Draw order follows child order: world, then slingshot, then HUD
        self.world = Node("World", parent=self)
        self.level = Level()
        self.world.add_child(self.level)
        self.add_child(Slingshot(self))
        self.add_child(HUD(self))   # last child, so it draws on top

    def _process(self, delta: float) -> None:
        if Input.is_action_just_pressed("debug_shapes"):
            Shape2D.debug_draw = not Shape2D.debug_draw
        if Input.is_action_just_pressed("restart"):
            self.level.queue_free()
            self.level = Level()
            self.world.add_child(self.level)


def main() -> None:
    loop = GameLoop(width=WIDTH, height=HEIGHT, title="Knockdown — nodepy-physics")
    loop.add_plugin(PhysicsServer(gravity=Vector2d(0, GRAVITY)))
    loop.set_scene(Game())
    loop.run()


if __name__ == "__main__":
    main()
