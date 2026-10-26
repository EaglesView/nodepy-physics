from __future__ import annotations

from typing import TYPE_CHECKING, Dict, List, Optional, Set, Tuple

import pymunk

from nodepy import Plugin, Vector2d

if TYPE_CHECKING:
    from nodepy import GameLoop
    from nodepy_physics.PhysicsBody2D import PhysicsBody2D

# Shapes of bodies with contact_monitor=True get this collision type, so the
# Python collision callbacks only run for contacts somebody listens to.
MONITORED = 1


class PhysicsServer(Plugin):
    """
    Owns the pymunk ``Space`` and steps it once per fixed physics tick.

    Install it before ``set_scene`` so bodies can register in ``_ready``.
    The space is stepped from ``on_post_physics``: after every node's
    ``_physics_process`` (where you apply forces and impulses), mirroring
    Godot's order.  Bodies then copy their new position and rotation back
    into the scene tree.

    Usage
    -----
    loop = GameLoop()
    loop.add_plugin(PhysicsServer(gravity=Vector2d(0, 900)))
    loop.set_scene(root)
    """

    name = "physics"

    _active: Optional["PhysicsServer"] = None

    def __init__(self, gravity: Optional[Vector2d] = None, iterations: int = 20,
                 sleep_time: float = 0.5) -> None:
        """
        Parameters
        ----------
        gravity     World gravity in px/s² (y points down).  Default (0, 900).
        iterations  Solver iterations per step.  Higher = stiffer stacks.
        sleep_time  Seconds a body must stay still before it sleeps.
                    Sleeping bodies cost nothing until something touches them.
        """
        gravity = gravity if gravity is not None else Vector2d(0, 900)
        self.space = pymunk.Space()
        self.space.gravity = (gravity.x, gravity.y)
        self.space.iterations = iterations
        self.space.sleep_time_threshold = sleep_time
        self.space.on_collision(MONITORED, None, begin=self._on_begin,
                                separate=self._on_separate)

        self._bodies: List["PhysicsBody2D"] = []
        self._owners: Dict[pymunk.Shape, "PhysicsBody2D"] = {}
        self._pending: List[Tuple[int, object]] = []
        self._live: Set[object] = set()   # queued or already in the space
        self._events: List[Tuple[str, "PhysicsBody2D", "PhysicsBody2D", float]] = []

    @classmethod
    def get(cls) -> "PhysicsServer":
        """Return the installed server, or raise a helpful error."""
        if cls._active is None:
            raise RuntimeError(
                "No PhysicsServer installed: call "
                "loop.add_plugin(PhysicsServer()) before loop.set_scene()"
            )
        return cls._active

    # ── Plugin hooks ─────────────────────────────────────────────────────────

    def on_install(self, loop: GameLoop) -> None:
        if PhysicsServer._active is not None:
            raise RuntimeError("Only one PhysicsServer can be installed at a time")
        PhysicsServer._active = self

    def on_post_physics(self, delta: float) -> None:
        self.step(delta)

    def on_shutdown(self) -> None:
        if PhysicsServer._active is self:
            PhysicsServer._active = None

    # ── Stepping ─────────────────────────────────────────────────────────────

    def step(self, delta: float) -> None:
        """Advance the simulation by *delta* seconds and sync the scene tree."""
        self._flush_pending()
        self.space.step(delta)
        for body in self._bodies:
            if body._body.body_type != pymunk.Body.STATIC and not body._body.is_sleeping:
                body._sync_from_physics()
        self._dispatch_events()

    def _flush_pending(self) -> None:
        # Bodies must be in the space before their shapes and joints, so
        # additions wait here and go in sorted by kind.
        if not self._pending:
            return
        self._pending.sort(key=lambda item: item[0])
        self.space.add(*(obj for _, obj in self._pending))
        self._pending.clear()

    # ── Registration (called by bodies, shapes and joints) ───────────────────

    def _queue(self, kind: int, obj: object) -> None:
        if obj not in self._live:
            self._live.add(obj)
            self._pending.append((kind, obj))

    def _add_body(self, node: "PhysicsBody2D") -> None:
        if node not in self._bodies:
            self._bodies.append(node)
        self._queue(0, node._body)

    def _remove_body(self, node: "PhysicsBody2D") -> None:
        if node not in self._bodies:
            return
        self._bodies.remove(node)
        body = node._body
        # Scan what we track rather than body.shapes / body.constraints, which
        # miss objects still waiting in _pending
        for obj in list(self._live):
            if isinstance(obj, pymunk.Shape) and obj.body is body:
                self._owners.pop(obj, None)
                self._remove(obj)
            elif isinstance(obj, pymunk.Constraint) and body in (obj.a, obj.b):
                self._remove(obj)
        self._remove(body)

    def _add_shape(self, shape: pymunk.Shape, owner: "PhysicsBody2D") -> None:
        self._owners[shape] = owner
        self._queue(1, shape)

    def _remove_shape(self, shape: pymunk.Shape) -> None:
        self._owners.pop(shape, None)
        self._remove(shape)

    def _add_constraint(self, constraint: pymunk.Constraint) -> None:
        self._queue(2, constraint)

    def _remove_constraint(self, constraint: pymunk.Constraint) -> None:
        self._remove(constraint)

    def _remove(self, obj: object) -> None:
        if obj not in self._live:
            return
        self._live.discard(obj)
        for i, (_, pending) in enumerate(self._pending):
            if pending is obj:
                del self._pending[i]
                return
        self.space.remove(obj)

    # ── Collision events ─────────────────────────────────────────────────────

    def _on_begin(self, arbiter: pymunk.Arbiter, space: pymunk.Space, data) -> None:
        a, b = (self._owners.get(s) for s in arbiter.shapes)
        if a is None or b is None:
            return
        speed = 0.0
        points = arbiter.contact_point_set.points
        if points:
            p = points[0].point_a
            rel = b._body.velocity_at_world_point(p) - a._body.velocity_at_world_point(p)
            speed = abs(rel.dot(arbiter.normal))
        # Signals are emitted after the step: the space is locked during it,
        # and listeners commonly free or spawn bodies.
        self._events.append(("entered", a, b, speed))

    def _on_separate(self, arbiter: pymunk.Arbiter, space: pymunk.Space, data) -> None:
        a, b = (self._owners.get(s) for s in arbiter.shapes)
        if a is None or b is None:
            return
        self._events.append(("exited", a, b, 0.0))

    def _dispatch_events(self) -> None:
        if not self._events:
            return
        batch = list(self._events)
        self._events.clear()
        for kind, a, b, speed in batch:
            for this, other in ((a, b), (b, a)):
                if not this.contact_monitor or this not in self._bodies:
                    continue
                if kind == "entered":
                    this.body_entered.emit(other, speed)
                else:
                    this.body_exited.emit(other)
