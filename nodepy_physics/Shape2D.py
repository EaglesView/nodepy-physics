from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import tkinter as tk
import pymunk

from nodepy import Matrix3x3, Node, Vector2d
from nodepy_physics.PhysicsBody2D import PhysicsBody2D
from nodepy_physics.PhysicsServer import MONITORED, PhysicsServer


class Shape2D(Node):
    """
    Collision geometry for its parent ``PhysicsBody2D``.

    The shape's own ``relative_pos`` and ``rotation`` offset it from the body
    when it enters the tree, so one body can carry several shapes.

    Shapes are invisible unless they get a ``color`` (useful for prototyping
    and simple art), or ``Shape2D.debug_draw`` is switched on to outline every
    shape in the scene.

    Parameters
    ----------
    density     Mass per px² of area; a body's mass is the sum over its shapes.
    friction    0 = ice, ~1 = rubber.
    elasticity  Bounciness, 0 = none, 1 = perfect bounce.
    color       Fill colour, or None to stay invisible.
    outline     Outline colour when drawn.
    """

    debug_draw: bool = False
    debug_color: str = "#4cc9f0"

    def __init__(self, name: str, density: float = 1.0, friction: float = 0.7,
                 elasticity: float = 0.1, color: Optional[str] = None,
                 outline: str = "", relative_pos: Optional[Vector2d] = None,
                 parent: Optional[Node] = None) -> None:
        self._shape: Optional[pymunk.Shape] = None
        self._drawn_style: Optional[Tuple[str, str]] = None
        self.density = density
        self.friction = friction
        self.elasticity = elasticity
        self.color = color
        self.outline = outline
        super().__init__(name, relative_pos=relative_pos, parent=parent)
        self.tree_entered.connect(self._on_tree_entered)
        self.tree_exited.connect(self._on_tree_exited)

    @property
    def pymunk_shape(self) -> Optional[pymunk.Shape]:
        """The underlying ``pymunk.Shape`` once the shape is in the tree."""
        return self._shape

    @property
    def body(self) -> Optional[PhysicsBody2D]:
        return self.parent if isinstance(self.parent, PhysicsBody2D) else None

    # ── Registration ─────────────────────────────────────────────────────────

    def _ready(self) -> None:
        self._attach()

    def _attach(self) -> None:
        body = self.body
        if body is None:
            raise RuntimeError(
                f"{type(self).__name__} {self.name!r} must be a child of a PhysicsBody2D")
        if self._shape is None:
            self._shape = self._make_shape(body.pymunk_body, self._local_matrix())
            self._shape.density = self.density
            self._shape.friction = self.friction
            self._shape.elasticity = self.elasticity
            if body.contact_monitor:
                self._shape.collision_type = MONITORED
        if body._in_space:
            PhysicsServer.get()._add_shape(self._shape, body)
        else:
            body._enter_physics()

    def _on_tree_entered(self, node: Node) -> None:
        if self._shape is not None and self.body is not None and self.body._in_space:
            PhysicsServer.get()._add_shape(self._shape, self.body)

    def _on_tree_exited(self, node: Node) -> None:
        if self._shape is not None and PhysicsServer._active is not None:
            PhysicsServer._active._remove_shape(self._shape)

    def _local_matrix(self) -> Matrix3x3:
        return Matrix3x3.make_transform(self.relative_pos, self.rotation,
                                        Vector2d(1, 1))

    def _make_shape(self, body: pymunk.Body, local: Matrix3x3) -> pymunk.Shape:
        raise NotImplementedError

    # ── Drawing ──────────────────────────────────────────────────────────────

    def _style(self) -> Optional[Tuple[str, str]]:
        """(fill, outline) to draw with, or None when the shape is invisible."""
        if self.color is not None:
            return self.color, self.outline
        if Shape2D.debug_draw:
            return "", Shape2D.debug_color
        return None

    def _outline_pts(self, cam: Matrix3x3) -> List[float]:
        raise NotImplementedError

    def _item_count(self) -> int:
        return 1

    def _items_current(self) -> bool:
        """True when the existing canvas items match the current style."""
        style = self._style()
        if style is None:
            return not self._canvas_ids
        return (style == self._drawn_style
                and len(self._canvas_ids) == self._item_count())

    def _update_draw(self, canvas: tk.Canvas, cam: Matrix3x3) -> bool:
        if not self._items_current():
            return False
        if self._canvas_ids:
            canvas.coords(self._canvas_ids[0], *self._outline_pts(cam))
        return True

    def _draw(self, canvas: tk.Canvas, cam: Matrix3x3) -> None:
        style = self._drawn_style = self._style()
        if style is None:
            return
        self._canvas_ids.append(canvas.create_polygon(
            self._outline_pts(cam), fill=style[0], outline=style[1], width=1))


class _PolygonalShape2D(Shape2D):
    """Shapes described by a list of local vertices."""

    def _vertices(self) -> Sequence[Tuple[float, float]]:
        raise NotImplementedError

    def _make_shape(self, body: pymunk.Body, local: Matrix3x3) -> pymunk.Shape:
        return pymunk.Poly(body, [local.multiply_vec(x, y) for x, y in self._vertices()])

    def _outline_pts(self, cam: Matrix3x3) -> List[float]:
        mat = cam * self.global_matrix
        pts: List[float] = []
        for x, y in self._vertices():
            pts.extend(mat.multiply_vec(x, y))
        return pts


class BoxShape2D(_PolygonalShape2D):
    """Axis-aligned box (before rotation), centred on the node."""

    def __init__(self, name: str, width: float, height: float, **kwargs) -> None:
        self.width = width
        self.height = height
        super().__init__(name, **kwargs)

    def _vertices(self) -> Sequence[Tuple[float, float]]:
        hw, hh = self.width / 2, self.height / 2
        return [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]


class PolygonShape2D(_PolygonalShape2D):
    """Convex polygon; concave point lists are wrapped in their convex hull."""

    def __init__(self, name: str, points: Sequence[Vector2d], **kwargs) -> None:
        self.points = [(p.x, p.y) for p in points]
        super().__init__(name, **kwargs)

    def _vertices(self) -> Sequence[Tuple[float, float]]:
        return self.points


class CircleShape2D(Shape2D):
    """Circle centred on the node.  Drawn with a spoke so rolling is visible."""

    def __init__(self, name: str, radius: float, **kwargs) -> None:
        self.radius = radius
        super().__init__(name, **kwargs)

    def _make_shape(self, body: pymunk.Body, local: Matrix3x3) -> pymunk.Shape:
        return pymunk.Circle(body, self.radius, local.multiply_vec(0, 0))

    def _item_count(self) -> int:
        return 2

    def _screen_circle(self, cam: Matrix3x3) -> Tuple[float, float, float, float, float]:
        mat = cam * self.global_matrix
        cx, cy = mat.multiply_vec(0, 0)
        ex, ey = mat.multiply_vec(self.radius, 0)
        return cx, cy, math.hypot(ex - cx, ey - cy), ex, ey

    def _outline_pts(self, cam: Matrix3x3) -> List[float]:
        cx, cy, r, _, _ = self._screen_circle(cam)
        return [cx - r, cy - r, cx + r, cy + r]

    def _update_draw(self, canvas: tk.Canvas, cam: Matrix3x3) -> bool:
        if not super()._update_draw(canvas, cam):
            return False
        if self._canvas_ids:
            cx, cy, _, ex, ey = self._screen_circle(cam)
            canvas.coords(self._canvas_ids[1], cx, cy, ex, ey)
        return True

    def _draw(self, canvas: tk.Canvas, cam: Matrix3x3) -> None:
        style = self._drawn_style = self._style()
        if style is None:
            return
        cx, cy, r, ex, ey = self._screen_circle(cam)
        spoke = style[1] or "#222222"
        self._canvas_ids.append(canvas.create_oval(
            cx - r, cy - r, cx + r, cy + r, fill=style[0], outline=style[1], width=1))
        self._canvas_ids.append(canvas.create_line(cx, cy, ex, ey, fill=spoke))


class SegmentShape2D(Shape2D):
    """Thick line from *a* to *b* (local space).  Great for ground and walls."""

    def __init__(self, name: str, a: Vector2d, b: Vector2d,
                 thickness: float = 2.0, **kwargs) -> None:
        self.a = a
        self.b = b
        self.thickness = thickness
        super().__init__(name, **kwargs)

    def _make_shape(self, body: pymunk.Body, local: Matrix3x3) -> pymunk.Shape:
        return pymunk.Segment(body, local.multiply_vec(self.a.x, self.a.y),
                              local.multiply_vec(self.b.x, self.b.y),
                              self.thickness / 2)

    def _outline_pts(self, cam: Matrix3x3) -> List[float]:
        mat = cam * self.global_matrix
        return [*mat.multiply_vec(self.a.x, self.a.y),
                *mat.multiply_vec(self.b.x, self.b.y)]

    def _draw(self, canvas: tk.Canvas, cam: Matrix3x3) -> None:
        style = self._drawn_style = self._style()
        if style is None:
            return
        self._canvas_ids.append(canvas.create_line(
            *self._outline_pts(cam), fill=style[0] or style[1],
            width=max(1, self.thickness)))
