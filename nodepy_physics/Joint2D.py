from __future__ import annotations

from typing import List, Optional, Tuple

import tkinter as tk
import pymunk

from nodepy import Matrix3x3, Node, Vector2d
from nodepy_physics.PhysicsBody2D import PhysicsBody2D
from nodepy_physics.PhysicsServer import PhysicsServer


class Joint2D(Node):
    """
    Connects two bodies.  Pass ``None`` as *body_b* to attach *body_a* to the
    world itself (a fixed point in space).

    Joints are drawn as a line between their anchors when given a ``color``.
    """

    def __init__(self, name: str, body_a: PhysicsBody2D,
                 body_b: Optional[PhysicsBody2D] = None,
                 color: Optional[str] = None, width: float = 2,
                 parent: Optional[Node] = None) -> None:
        self.body_a = body_a
        self.body_b = body_b
        self.color = color
        self.width = width
        self._constraint: Optional[pymunk.Constraint] = None
        super().__init__(name, parent=parent)
        self.tree_entered.connect(self._on_tree_entered)
        self.tree_exited.connect(self._on_tree_exited)

    @property
    def pymunk_constraint(self) -> Optional[pymunk.Constraint]:
        return self._constraint

    def _pymunk_bodies(self) -> Tuple[pymunk.Body, pymunk.Body]:
        server = PhysicsServer.get()
        b = self.body_b.pymunk_body if self.body_b is not None else server.space.static_body
        return self.body_a.pymunk_body, b

    def _ready(self) -> None:
        a, b = self._pymunk_bodies()
        self._constraint = self._make_constraint(a, b)
        PhysicsServer.get()._add_constraint(self._constraint)

    def _make_constraint(self, a: pymunk.Body, b: pymunk.Body) -> pymunk.Constraint:
        raise NotImplementedError

    def _on_tree_entered(self, node: Node) -> None:
        if self._constraint is not None:
            PhysicsServer.get()._add_constraint(self._constraint)

    def _on_tree_exited(self, node: Node) -> None:
        if self._constraint is not None and PhysicsServer._active is not None:
            PhysicsServer._active._remove_constraint(self._constraint)

    # ── Drawing ──────────────────────────────────────────────────────────────

    def _world_anchors(self) -> Optional[List[float]]:
        """Line endpoints in world space, or None when there is nothing to draw."""
        c = self._constraint
        if c is None or not hasattr(c, "anchor_a"):
            return None
        ax, ay = c.a.local_to_world(c.anchor_a)
        bx, by = c.b.local_to_world(c.anchor_b)
        return [ax, ay, bx, by]

    def _screen_pts(self, cam: Matrix3x3) -> Optional[List[float]]:
        pts = self._world_anchors()
        if pts is None:
            return None
        return [*cam.multiply_vec(pts[0], pts[1]), *cam.multiply_vec(pts[2], pts[3])]

    def _update_draw(self, canvas: tk.Canvas, cam: Matrix3x3) -> bool:
        if self.color is None:
            return not self._canvas_ids
        pts = self._screen_pts(cam)
        if pts is None or len(self._canvas_ids) != 1:
            return False
        canvas.coords(self._canvas_ids[0], *pts)
        return True

    def _draw(self, canvas: tk.Canvas, cam: Matrix3x3) -> None:
        pts = self._screen_pts(cam) if self.color is not None else None
        if pts is not None:
            self._canvas_ids.append(
                canvas.create_line(*pts, fill=self.color, width=self.width))


def _vec(v: Optional[Vector2d]) -> Tuple[float, float]:
    return (v.x, v.y) if v is not None else (0.0, 0.0)


class PinJoint2D(Joint2D):
    """
    Hinge: both bodies rotate freely around one shared point, the joint
    node's own position (world space) when it enters the tree.
    """

    def _make_constraint(self, a: pymunk.Body, b: pymunk.Body) -> pymunk.Constraint:
        p = self.relative_pos
        return pymunk.PivotJoint(a, b, (p.x, p.y))


class RopeJoint2D(Joint2D):
    """
    Keeps the anchors at most *length* apart (slack allowed), like a rope.
    Anchors are local to each body (world space when the body is None).
    Without *length*, the current distance is used.
    """

    def __init__(self, name: str, body_a: PhysicsBody2D,
                 body_b: Optional[PhysicsBody2D] = None,
                 anchor_a: Optional[Vector2d] = None,
                 anchor_b: Optional[Vector2d] = None,
                 length: Optional[float] = None, **kwargs) -> None:
        self.anchor_a = anchor_a
        self.anchor_b = anchor_b
        self.length = length
        super().__init__(name, body_a, body_b, **kwargs)

    def _make_constraint(self, a: pymunk.Body, b: pymunk.Body) -> pymunk.Constraint:
        aa, ab = _vec(self.anchor_a), _vec(self.anchor_b)
        length = self.length
        if length is None:
            length = abs(a.local_to_world(aa) - b.local_to_world(ab))
        return pymunk.SlideJoint(a, b, aa, ab, 0, length)


class DampedSpringJoint2D(Joint2D):
    """A spring between two anchors (local to each body)."""

    def __init__(self, name: str, body_a: PhysicsBody2D,
                 body_b: Optional[PhysicsBody2D] = None,
                 anchor_a: Optional[Vector2d] = None,
                 anchor_b: Optional[Vector2d] = None,
                 rest_length: Optional[float] = None, stiffness: float = 50.0,
                 damping: float = 1.0, **kwargs) -> None:
        self.anchor_a = anchor_a
        self.anchor_b = anchor_b
        self.rest_length = rest_length
        self.stiffness = stiffness
        self.damping = damping
        super().__init__(name, body_a, body_b, **kwargs)

    def _make_constraint(self, a: pymunk.Body, b: pymunk.Body) -> pymunk.Constraint:
        aa, ab = _vec(self.anchor_a), _vec(self.anchor_b)
        rest = self.rest_length
        if rest is None:
            rest = abs(a.local_to_world(aa) - b.local_to_world(ab))
        return pymunk.DampedSpring(a, b, aa, ab, rest, self.stiffness, self.damping)
