from __future__ import annotations

import math
from typing import Optional

import pymunk

from nodepy import Node, Signal, Vector2d
from nodepy_physics.PhysicsServer import PhysicsServer


class PhysicsBody2D(Node):
    """
    Base class for nodes simulated by the ``PhysicsServer``.

    Give a body one or more shape children (``CircleShape2D``,
    ``BoxShape2D``, ...) — the shapes register the body when they enter the
    tree, so you are free to override ``_ready`` without calling ``super()``.

    Place bodies directly under an untransformed node (your "world"):
    ``relative_pos`` and ``rotation`` are treated as world coordinates.
    Setting either one teleports the body.

    Signals (only when ``contact_monitor`` is True)
    -------
    body_entered(other, impact_speed)   another body started touching this one
    body_exited(other)                  it stopped touching
    """

    _body_type = pymunk.Body.DYNAMIC

    def __init__(self, name: str, relative_pos: Optional[Vector2d] = None,
                 rotation: float = 0.0, contact_monitor: bool = False,
                 parent: Optional[Node] = None) -> None:
        self._body = pymunk.Body(body_type=self._body_type)
        self._in_space = False
        self._entered_once = False
        self._syncing = False
        self.contact_monitor = contact_monitor
        self.body_entered = Signal("body_entered")
        self.body_exited = Signal("body_exited")
        super().__init__(name, relative_pos=relative_pos, parent=parent)
        self.rotation = rotation
        self._push_transform()
        self.tree_entered.connect(self._on_tree_entered)
        self.tree_exited.connect(self._on_tree_exited)

    @property
    def pymunk_body(self) -> pymunk.Body:
        """The underlying ``pymunk.Body``, for anything this API does not cover."""
        return self._body

    # ── Transform (writes teleport the body) ─────────────────────────────────

    @property
    def relative_pos(self) -> Vector2d:
        return self._relative_pos

    @relative_pos.setter
    def relative_pos(self, value: Vector2d) -> None:
        Node.relative_pos.fset(self, value)
        self._push_transform()

    @property
    def rotation(self) -> float:
        return self._rotation

    @rotation.setter
    def rotation(self, value: float) -> None:
        Node.rotation.fset(self, value)
        self._push_transform()

    def _push_transform(self) -> None:
        if self._syncing or not hasattr(self, "_body"):
            return
        self._body.position = (self._relative_pos.x, self._relative_pos.y)
        self._body.angle = math.radians(self._rotation)
        if self._in_space:
            if self._body.body_type == pymunk.Body.STATIC:
                PhysicsServer.get().space.reindex_shapes_for_body(self._body)
            else:
                self._body.activate()

    def _sync_from_physics(self) -> None:
        x, y = self._body.position
        self._syncing = True
        try:
            self.relative_pos = Vector2d(x, y)
            self.rotation = math.degrees(self._body.angle)
        finally:
            self._syncing = False

    # ── Space membership ─────────────────────────────────────────────────────

    def _enter_physics(self) -> None:
        if self._in_space:
            return
        self._in_space = True
        self._entered_once = True
        server = PhysicsServer.get()
        server._add_body(self)
        from nodepy_physics.Shape2D import Shape2D
        for child in self.children:
            if isinstance(child, Shape2D) and child._shape is not None:
                server._add_shape(child._shape, self)

    def _exit_physics(self) -> None:
        if not self._in_space:
            return
        self._in_space = False
        if PhysicsServer._active is not None:
            PhysicsServer._active._remove_body(self)

    def _on_tree_entered(self, node: Node) -> None:
        # Re-entering after a remove_child; the first entry happens via shapes
        if self._entered_once:
            self._enter_physics()

    def _on_tree_exited(self, node: Node) -> None:
        self._exit_physics()


class StaticBody2D(PhysicsBody2D):
    """An immovable body: ground, walls, platforms."""

    _body_type = pymunk.Body.STATIC


class _MovingBody2D(PhysicsBody2D):
    """Velocity accessors shared by kinematic and rigid bodies."""

    @property
    def linear_velocity(self) -> Vector2d:
        return Vector2d(*self._body.velocity)

    @linear_velocity.setter
    def linear_velocity(self, value: Vector2d) -> None:
        self._body.velocity = (value.x, value.y)

    @property
    def angular_velocity(self) -> float:
        """Degrees per second, clockwise."""
        return math.degrees(self._body.angular_velocity)

    @angular_velocity.setter
    def angular_velocity(self, value: float) -> None:
        self._body.angular_velocity = math.radians(value)


class KinematicBody2D(_MovingBody2D):
    """
    Moved by your code, not by forces.  Set ``linear_velocity`` (or teleport
    it) and dynamic bodies are pushed out of the way: moving platforms, doors.
    """

    _body_type = pymunk.Body.KINEMATIC


class RigidBody2D(_MovingBody2D):
    """
    A body driven by the simulation: gravity, collisions, joints, impulses.
    Its mass comes from the ``density`` of its shapes.
    """

    _body_type = pymunk.Body.DYNAMIC

    @property
    def mass(self) -> float:
        return self._body.mass

    @property
    def sleeping(self) -> bool:
        return self._in_space and self._body.is_sleeping

    def wake_up(self) -> None:
        if self._in_space:
            self._body.activate()

    def apply_impulse(self, impulse: Vector2d,
                      offset: Optional[Vector2d] = None) -> None:
        """Instant push in world space, at *offset* from the centre (local space)."""
        self._body.apply_impulse_at_world_point((impulse.x, impulse.y),
                                                self._world_point(offset))
        self.wake_up()

    def apply_force(self, force: Vector2d,
                    offset: Optional[Vector2d] = None) -> None:
        """Continuous push in world space for this step; call from _physics_process."""
        self._body.apply_force_at_world_point((force.x, force.y),
                                              self._world_point(offset))
        self.wake_up()

    def _world_point(self, offset: Optional[Vector2d]) -> pymunk.Vec2d:
        local = (offset.x, offset.y) if offset is not None else (0, 0)
        return self._body.local_to_world(local)
