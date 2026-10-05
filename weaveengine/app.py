"""WeaveEngine desktop application (design section 23).

Import a DSN, watch it being routed, export the SES. The window never routes
itself: a ``session.Job`` does that in another process and this module draws
what it reports.
"""
import io
import math
import os
import sys
import threading
import time

from PySide6.QtCore import QEvent, QPoint, QPointF, QRectF, Qt, QTimer, Signal
from shapely.geometry import Point
from PySide6.QtGui import QAction, QBrush, QColor, QKeySequence, QPainter, QPainterPath, QPen, QPolygonF
from PySide6.QtWidgets import (QApplication, QCheckBox, QDialog, QDialogButtonBox, QDockWidget, QDoubleSpinBox,
                               QFileDialog, QFormLayout, QFrame, QGraphicsScene, QGraphicsView, QGridLayout,
                               QHBoxLayout, QLabel, QMainWindow, QMessageBox, QPlainTextEdit, QProgressBar,
                               QPushButton, QSpinBox, QTabWidget, QVBoxLayout, QWidget)

from weaveengine import accel
from weaveengine.io.check import measure
from weaveengine.io.dsn import read_dsn
from weaveengine.io.ses import write_ses
from weaveengine.progress import ProgressBar, clock
from weaveengine.session import Job
from weaveengine.settings import AUTOMATIC, DESCRIPTIONS, Settings, default_path
from weaveengine.viz.svg import export_result

LAYER_COLORS = ["#e0483c", "#3c7be0", "#35b56a", "#c9a227", "#b06cff", "#22cfcf"]
BACKGROUND = "#121212"
BOARD = "#1b2a1f"


class BoardView(QGraphicsView):
    """Picture of the board and its traces. Scene units are millimetres.

    Scroll (wheel or two fingers) or pinch: zoom, about the point under the
    cursor, by an amount proportional to the movement. Right or middle button
    drag: pan. Left click: select what is under the cursor.
    """

    selected = Signal(object)  # dict describing the selection, or None

    ZOOM_PER_WHEEL_UNIT = 0.0015   # a mouse-wheel notch (120 units) is about 20 %
    ZOOM_PER_PIXEL = 0.004         # trackpad: 100 pixels of travel is about 50 %

    def __init__(self):
        super().__init__()
        self.setScene(QGraphicsScene(self))
        self.setBackgroundBrush(QColor(BACKGROUND))
        self.setRenderHint(QPainter.Antialiasing, True)
        self.setFrameShape(QFrame.NoFrame)
        # The view's own transform does all the moving: no scroll bars, no
        # automatic anchoring (which fights the cursor once a scroll bar hits its end).
        self.setDragMode(QGraphicsView.NoDrag)
        self.setTransformationAnchor(QGraphicsView.NoAnchor)
        self.setResizeAnchor(QGraphicsView.NoAnchor)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setViewportUpdateMode(QGraphicsView.SmartViewportUpdate)
        self.setOptimizationFlag(QGraphicsView.DontSavePainterState, True)
        self.setOptimizationFlag(QGraphicsView.DontAdjustForAntialiasing, True)
        self.setContextMenuPolicy(Qt.NoContextMenu)
        # While the picture is moving it is drawn without smoothing; shortly
        # after it stops, once more with.
        self._settle = QTimer(self)
        self._settle.setSingleShot(True)
        self._settle.setInterval(140)
        self._settle.timeout.connect(self._settled)
        self.trace_items: list = []     # (item, width in mm, colour)
        self.detail_items: list = []    # teardrops: only worth drawing when zoomed in
        self._thin = None               # whether traces are currently drawn as hairlines
        self.viewport().grabGesture(Qt.PinchGesture)
        self.board = None
        self.visible: dict[int, bool] = {}
        self.show_outlines = True
        self.show_names = True
        self.static_items: list = []
        self.live_items: list = []
        self.outline_items: list = []
        self.name_items: list = []
        self.selection_items: list = []
        self.selection: dict | None = None
        self.last = None      # what is on screen, so it can be redrawn when a layer is toggled
        self.wire_info: dict = {}  # extra facts per wire for the selection panel: index in ``wires`` -> dict
        self._pan_from = None
        self._fit_scale = 1.0

    # -- the board itself -----------------------------------------------------
    def set_board(self, board) -> None:
        self.scene().clear()
        self.static_items, self.live_items, self.last = [], [], None
        self.outline_items, self.name_items, self.selection_items = [], [], []
        self.trace_items, self.detail_items, self._thin = [], [], None
        self.pad_items, self.selection = [], None
        self.board = board
        self.visible = {i: True for i in range(len(board.layers))}
        scene = self.scene()
        x0, y0, x1, y1 = board.outline.bounds
        span = max(x1 - x0, y1 - y0)
        scene.setSceneRect(QRectF(x0 - 20 * span, -y1 - 20 * span, 41 * span, 41 * span))  # room to pan freely
        outline = QPolygonF([QPointF(x, -y) for x, y in board.outline.exterior.coords])
        scene.addPolygon(outline, QPen(QColor("#7a8a7d"), 0), QBrush(QColor(BOARD))).setZValue(-10)
        for obs in board.obstacles:
            poly = QPolygonF([QPointF(x, -y) for x, y in obs.shape.exterior.coords])
            pen = QPen(QColor("#ff4d4d"), 0, Qt.DashLine)
            scene.addPolygon(poly, pen, QBrush(QColor(58, 23, 23, 120))).setZValue(-5)
        for pad in board.pads:
            self._add_pad(pad)
        pen = QPen(QColor("#9fb3a5"), 0)
        for comp in board.components:
            path = QPainterPath()
            for line in comp.outlines:
                path.moveTo(line[0][0], -line[0][1])
                for x, y in line[1:]:
                    path.lineTo(x, -y)
            item = scene.addPath(path, pen)
            item.setZValue(5)
            item.setVisible(self.show_outlines)
            self.outline_items.append(item)
            box = comp.bounds()
            cx, cy, size = ((box[0] + box[2]) / 2, (box[1] + box[3]) / 2, min(box[2] - box[0], box[3] - box[1])) if box else (comp.x, comp.y, 2.0)
            text = scene.addSimpleText(comp.reference)
            text.setBrush(QBrush(QColor("#e8efe9")))
            height = min(1.6, max(0.6, 0.3 * size))          # text height in mm
            rect = text.boundingRect()
            k = height / rect.height()
            text.setScale(k)
            text.setPos(cx - rect.width() * k / 2, -cy - rect.height() * k / 2)
            text.setZValue(65)
            text.setVisible(self.show_names)
            self.name_items.append((text, height))
        self.fit()

    def set_outlines_visible(self, on: bool) -> None:
        self.show_outlines = on
        self._level_of_detail(force=True)
        if not on:
            self._drop_selection("part")

    def set_names_visible(self, on: bool) -> None:
        self.show_names = on
        self._level_of_detail(force=True)

    # -- drawing speed ------------------------------------------------------------
    def _level_of_detail(self, force: bool = False) -> None:
        """Match what is drawn to the zoom: detail too small to see costs time and adds nothing."""
        scale = self.transform().m11()          # pixels per millimetre
        for text, height in self.name_items:
            text.setVisible(self.show_names and height * scale >= 7.0)        # legible, or not drawn
        for item in self.outline_items:
            item.setVisible(self.show_outlines and scale >= 2.0)
        for item in self.detail_items:
            item.setVisible(scale >= 6.0)
        rules = self.board.rules if self.board is not None else None
        thin = rules is not None and rules.base_width * scale < 1.6          # traces about a pixel wide
        if thin != self._thin or force:
            self._thin = thin
            for item, width, color in self.trace_items:
                if width * scale < 1.6:
                    pen = QPen(color, 0)       # a hairline: by far the fastest thing to draw
                else:
                    pen = QPen(color, width, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)
                item.setPen(pen)

    def _moving(self) -> None:
        if self.renderHints() & QPainter.Antialiasing:
            self.setRenderHint(QPainter.Antialiasing, False)
        self._settle.start()

    def _settled(self) -> None:
        self.setRenderHint(QPainter.Antialiasing, True)
        self.viewport().update()

    def _add_pad(self, pad, live: bool = False) -> None:
        poly = QPolygonF([QPointF(x, -y) for x, y in pad.shape.exterior.coords])
        if pad.is_via:
            fill = "#b8c2cc"
        elif pad.layers is None:
            fill = "#d9b23c"
        else:
            fill = QColor(LAYER_COLORS[min(pad.layers) % len(LAYER_COLORS)]).lighter(150).name()
        item = self.scene().addPolygon(poly, QPen(Qt.NoPen), QBrush(QColor(fill)))
        item.setZValue(50)
        (self.live_items if live else self.static_items).append(item)
        if not live:
            item.setVisible(self._pad_shown(pad))
            self.pad_items.append((item, pad))
        if pad.is_via:
            r = self.board.rules.via_drill / 2.0
            x, y = pad.centre
            hole = self.scene().addEllipse(QRectF(x - r, -y - r, 2 * r, 2 * r), QPen(Qt.NoPen), QBrush(QColor(BACKGROUND)))
            hole.setZValue(51)
            (self.live_items if live else self.static_items).append(hole)

    # -- moving around ----------------------------------------------------------
    def fit(self) -> None:
        """Show the whole board."""
        if self.board is None:
            return
        x0, y0, x1, y1 = self.board.outline.bounds
        pad = 0.03 * max(x1 - x0, y1 - y0)
        w, h = max(1, self.viewport().width()), max(1, self.viewport().height())
        scale = min(w / (x1 - x0 + 2 * pad), h / (y1 - y0 + 2 * pad))
        self._fit_scale = scale
        self.resetTransform()
        self.scale(scale, scale)
        self.centerOn(QPointF((x0 + x1) / 2, -(y0 + y1) / 2))
        self._level_of_detail()

    def zoom_at(self, view_pos, factor: float) -> None:
        """Zoom by ``factor`` keeping the scene point under ``view_pos`` (viewport pixels) where it is."""
        current = self.transform().m11()
        lo, hi = self._fit_scale / 4.0, max(self._fit_scale * 400.0, 2000.0)  # a quarter of the board .. about half a micron per pixel
        factor = max(lo / current, min(hi / current, factor))
        if abs(factor - 1.0) < 1e-9:
            return
        self._moving()
        before = self.mapToScene(view_pos)
        self.scale(factor, factor)
        after = self.mapToScene(view_pos)
        self.translate(after.x() - before.x(), after.y() - before.y())
        self._level_of_detail()

    def pan_by(self, dx: float, dy: float) -> None:
        """Move the picture by this many viewport pixels."""
        self._moving()
        k = self.transform().m11()
        self.translate(dx / k, dy / k)

    def wheelEvent(self, event) -> None:
        pixels = event.pixelDelta()
        if not pixels.isNull():   # a trackpad: many small steps, so scale with the distance moved
            factor = math.exp(self.ZOOM_PER_PIXEL * pixels.y())
        else:                     # a wheel
            factor = math.exp(self.ZOOM_PER_WHEEL_UNIT * event.angleDelta().y())
        self.zoom_at(event.position().toPoint(), factor)
        event.accept()

    def viewportEvent(self, event) -> bool:
        if event.type() == QEvent.Gesture:
            pinch = event.gesture(Qt.PinchGesture)
            if pinch is not None:
                self.zoom_at(self.viewport().mapFromGlobal(pinch.centerPoint().toPoint()), pinch.scaleFactor())
                return True
        if event.type() == QEvent.NativeGesture and event.gestureType() == Qt.ZoomNativeGesture:
            self.zoom_at(event.position().toPoint(), 1.0 + event.value())
            return True
        return super().viewportEvent(event)

    def mousePressEvent(self, event) -> None:
        if event.button() in (Qt.RightButton, Qt.MiddleButton):
            self._pan_from = event.position()
            self.viewport().setCursor(Qt.ClosedHandCursor)
        elif event.button() == Qt.LeftButton:
            self.select_at(self.mapToScene(event.position().toPoint()))
        event.accept()

    def mouseMoveEvent(self, event) -> None:
        if self._pan_from is not None:
            now = event.position()
            self.pan_by(now.x() - self._pan_from.x(), now.y() - self._pan_from.y())
            self._pan_from = now
        event.accept()

    def mouseReleaseEvent(self, event) -> None:
        if event.button() in (Qt.RightButton, Qt.MiddleButton):
            self._pan_from = None
            self.viewport().unsetCursor()
        event.accept()

    # -- selection ----------------------------------------------------------------
    def select_at(self, scene_pos) -> dict | None:
        """What is at this scene point: a pad, else a trace, else a part's outline. Highlights it and emits ``selected``."""
        info = self.selection = self.pick(scene_pos.x(), -scene_pos.y())
        self.highlight(info)
        self.selected.emit(info)
        return info

    def _pad_shown(self, pad) -> bool:
        """A pad is drawn while any layer it is on is shown (a through-hole pad is on all of them)."""
        layers = range(len(self.board.layers)) if pad.layers is None else pad.layers
        return any(self.visible.get(layer, True) for layer in layers)

    def _drop_selection(self, *kinds: str) -> None:
        """Clears the selection if it is one of ``kinds`` and has just been hidden."""
        info = self.selection
        if info is None or info["kind"] not in kinds:
            return
        if info["kind"] == "trace" and self.visible.get(info["layer index"], True):
            return
        if info["kind"] in ("pad", "via") and self._pad_shown(info["pad"]):
            return
        self.selection = None
        self.highlight(None)
        self.selected.emit(None)

    def pick(self, x: float, y: float) -> dict | None:
        """Only what is switched on can be picked (design section 23). Detail
        that is merely too small to draw at this zoom still can."""
        board = self.board
        if board is None:
            return None
        reach = 5.0 / self.transform().m11()   # five pixels, in mm
        point = Point(x, y)
        names = board.net_names
        for pad in board.pads:
            if self._pad_shown(pad) and pad.shape.contains(point):
                owner = next((c for c in board.components if pad.pad_id in c.pads), None)
                bx0, by0, bx1, by1 = pad.shape.bounds
                return {"kind": "via" if pad.is_via else "pad", "name": pad.name or str(pad.pad_id),
                        "net": names.get(pad.net_id, "") if pad.net_id >= 0 else "(not connected)",
                        "layers": "all" if pad.layers is None else ", ".join(board.layers[i] for i in sorted(pad.layers)),
                        "size": f"{bx1 - bx0:.2f} × {by1 - by0:.2f} mm",
                        "part": f"{owner.reference} ({owner.value})" if owner and owner.value else (owner.reference if owner else ""),
                        "outline": [list(pad.shape.exterior.coords)], "pad": pad}
        wires = self.last[0] if self.last else []
        best = None
        for index, (layer, net, pts) in enumerate(wires):
            if not self.visible.get(layer, True):
                continue
            limit = max(reach, board.rules.width(net) / 2.0)
            for (ax, ay), (bx, by) in zip(pts, pts[1:]):
                if min(ax, bx) - limit > x or max(ax, bx) + limit < x or min(ay, by) - limit > y or max(ay, by) + limit < y:
                    continue
                dx, dy = bx - ax, by - ay
                n2 = dx * dx + dy * dy
                t = 0.0 if n2 == 0 else min(1.0, max(0.0, ((x - ax) * dx + (y - ay) * dy) / n2))
                d = math.hypot(x - ax - t * dx, y - ay - t * dy)
                if d <= limit and (best is None or d < best[0] or (d == best[0] and layer < wires[best[1]][0])):
                    best = (d, index)
        if best is not None:
            layer, net, pts = wires[best[1]]
            length = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
            same = [w for w in wires if w[1] == net]
            info = {"kind": "trace", "net": names.get(net, str(net)), "layer": board.layers[layer],
                    "length": f"{length:.2f} mm" + (" (rough, still routing)" if self.last[5] else ""),
                    "width": f"{board.rules.width(net):g} mm",
                    "net total": f"{sum(math.dist(a, b) for w in same for a, b in zip(w[2], w[2][1:])):.2f} mm in {len(same)} trace(s)",
                    "outline": [pts], "trace": True, "layer index": layer}
            info.update(self.wire_info.get(best[1], {}))
            return info
        found = None
        for comp in board.components if self.show_outlines else ():
            box = comp.bounds()
            if box and box[0] <= x <= box[2] and box[1] <= y <= box[3]:
                area = (box[2] - box[0]) * (box[3] - box[1])
                if found is None or area < found[0]:
                    found = (area, comp)
        if found is not None:
            comp = found[1]
            return {"kind": "part", "name": comp.reference, "value": comp.value or "(none)", "footprint": comp.footprint,
                    "side": comp.side, "position": f"{comp.x:.2f}, {comp.y:.2f} mm", "rotation": f"{comp.rotation:g}°",
                    "pads": str(len(comp.pads)), "outline": comp.outlines}
        return None

    def highlight(self, info: dict | None) -> None:
        for item in self.selection_items:
            self.scene().removeItem(item)
        self.selection_items = []
        if not info:
            return
        path = QPainterPath()
        for line in info["outline"]:
            path.moveTo(line[0][0], -line[0][1])
            for x, y in line[1:]:
                path.lineTo(x, -y)
        pen = QPen(QColor("#ffe45c"), 0)
        pen.setWidth(3)
        pen.setCosmetic(True)   # three pixels at any zoom
        item = self.scene().addPath(path, pen)
        item.setZValue(90)
        self.selection_items.append(item)

    # -- traces ---------------------------------------------------------------
    def show_routing(self, wires, open_pairs=(), vias=(), teardrops=(), violations=(), rough: bool = False,
                     changed=()) -> None:
        """``wires``: (layer, net, points). ``teardrops``: (layer, outline). ``vias``: centres or Pad objects.
        ``changed``: wires just placed or rerouted, drawn highlighted on top."""
        self.last = (wires, open_pairs, vias, teardrops, violations, rough, changed)
        scene = self.scene()
        for item in self.live_items:
            scene.removeItem(item)
        self.live_items = []
        if self.board is None:
            return
        rules = self.board.rules
        self.trace_items, self.detail_items = [], []
        layers = len(self.board.layers)
        for layer, net, pts in wires:
            if not self.visible.get(layer, True) or len(pts) < 2:
                continue
            path = QPainterPath()
            path.moveTo(pts[0][0], -pts[0][1])
            for x, y in pts[1:]:
                path.lineTo(x, -y)
            color = QColor(LAYER_COLORS[layer % len(LAYER_COLORS)])
            color.setAlpha(150 if rough else 225)
            item = scene.addPath(path)
            item.setZValue(10 + (layers - layer))     # back layers below, the front on top
            self.live_items.append(item)
            self.trace_items.append((item, rules.width(net), color))
        if changed:
            glow = QPainterPath()
            for layer, _, pts in changed:
                if self.visible.get(layer, True) and len(pts) >= 2:
                    glow.moveTo(pts[0][0], -pts[0][1])
                    for x, y in pts[1:]:
                        glow.lineTo(x, -y)
            item = scene.addPath(glow, QPen(QColor(255, 255, 200, 230), rules.base_width * 0.6, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            item.setZValue(40)
            self.live_items.append(item)
        drops: dict[int, QPainterPath] = {}
        for layer, outline in teardrops:
            if self.visible.get(layer, True):
                drops.setdefault(layer, QPainterPath()).addPolygon(QPolygonF([QPointF(x, -y) for x, y in outline] + [QPointF(outline[0][0], -outline[0][1])]))
        for layer, path in drops.items():
            color = QColor(LAYER_COLORS[layer % len(LAYER_COLORS)])
            color.setAlpha(225)
            path.setFillRule(Qt.WindingFill)
            item = scene.addPath(path, QPen(Qt.NoPen), QBrush(color))
            item.setZValue(10 + (layers - layer))
            self.live_items.append(item)
            self.detail_items.append(item)
        if open_pairs:
            air = QPainterPath()
            for (ax, ay), (bx, by) in open_pairs:
                air.moveTo(ax, -ay)
                air.lineTo(bx, -by)
            item = scene.addPath(air, QPen(QColor(255, 255, 255, 170), 0, Qt.DashLine))
            item.setZValue(60)
            self.live_items.append(item)
        known = {p.centre for p in self.board.pads}
        for via in vias:
            if hasattr(via, "shape"):
                if via.centre not in known:
                    self._add_pad(via, live=True)
            elif tuple(via) not in known:
                r = rules.via_diameter / 2.0
                item = scene.addEllipse(QRectF(via[0] - r, -via[1] - r, 2 * r, 2 * r), QPen(Qt.NoPen), QBrush(QColor("#b8c2cc")))
                item.setZValue(50)
                self.live_items.append(item)
        for v in violations:
            r = 4 * rules.pitch
            item = scene.addEllipse(QRectF(v.at[0] - r, -v.at[1] - r, 2 * r, 2 * r), QPen(QColor("#ff00ff"), 0))
            item.setZValue(70)
            self.live_items.append(item)

        self._level_of_detail(force=True)

    def set_layer_visible(self, layer: int, on: bool) -> None:
        self.visible[layer] = on
        for item, pad in self.pad_items:
            item.setVisible(self._pad_shown(pad))
        if self.last is not None:
            self.show_routing(*self.last)
        if not on:
            self._drop_selection("trace", "pad", "via")


class SettingsDialog(QDialog):
    """Editor for ``settings.json``: one tab per group, one row per setting.

    On/off settings are switches. A number that can also be left to the program
    ("use every core", "use the DSN's value") is a switch for that plus the
    number, which is only enabled when the switch is off.
    """

    def __init__(self, settings: Settings, path: str, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Settings")
        self.path = path
        self.readers: dict[str, object] = {}   # name -> function returning the value
        self.writers: dict[str, object] = {}   # name -> function taking a value
        tabs = QTabWidget()
        forms: dict[str, QFormLayout] = {}
        for name, (label, description, group) in DESCRIPTIONS.items():
            if group not in forms:
                page = QWidget()
                forms[group] = QFormLayout(page)
                forms[group].setVerticalSpacing(4)
                tabs.addTab(page, group)
            form = forms[group]
            value = getattr(Settings(), name)
            if name == "portfolio":
                self._race_row(form, label, description)
            elif isinstance(value, bool):
                box = QCheckBox(label)
                box.setToolTip(description)
                form.addRow(box)
                self.readers[name], self.writers[name] = box.isChecked, box.setChecked
            elif name in AUTOMATIC:
                self._automatic_row(form, name, label, description, isinstance(value, int))
            else:
                spin = self._spin(isinstance(value, int), minimum=0)
                form.addRow(label, spin)
                self.readers[name], self.writers[name] = spin.value, spin.setValue
            hint = QLabel(description)
            hint.setStyleSheet("color: gray; font-size: 11px; margin-bottom: 6px;")
            hint.setWordWrap(True)
            form.addRow(hint)
        self.load(settings)
        buttons = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel | QDialogButtonBox.RestoreDefaults)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        buttons.button(QDialogButtonBox.RestoreDefaults).clicked.connect(self.restore_defaults)
        layout = QVBoxLayout(self)
        layout.addWidget(tabs)
        where = QLabel(f"Saved to {path}")
        where.setStyleSheet("color: gray; font-size: 11px;")
        where.setWordWrap(True)
        layout.addWidget(where)
        layout.addWidget(buttons)
        self.resize(520, 640)

    @staticmethod
    def _spin(whole: bool, minimum: float = 0):
        if whole:
            spin = QSpinBox()
            spin.setRange(int(minimum), 100000)
        else:
            spin = QDoubleSpinBox()
            spin.setDecimals(3)
            spin.setRange(minimum, 1000.0)
            spin.setSingleStep(0.05)
        return spin

    def _automatic_row(self, form, name: str, label: str, description: str, whole: bool) -> None:
        """A switch for "let the program decide" and the number used otherwise (stored as 0 = automatic)."""
        text, manual = AUTOMATIC[name]
        auto = QCheckBox(text)
        spin = self._spin(whole, minimum=1 if whole else 0.001)
        spin.setValue(manual)
        auto.toggled.connect(lambda on: spin.setEnabled(not on))
        row = QHBoxLayout()
        row.addWidget(auto, 1)
        row.addWidget(spin)
        form.addRow(label, row)

        def write(value):
            auto.setChecked(value == 0)
            spin.setEnabled(value != 0)
            if value:
                spin.setValue(value)

        self.readers[name] = lambda: 0 if auto.isChecked() else spin.value()
        self.writers[name] = write

    def _race_row(self, form, label: str, description: str) -> None:
        """Racing is on or off; when on, the number of variants is automatic or chosen (stored as 1 = off, 0 = automatic)."""
        text, manual = AUTOMATIC["portfolio"]
        race = QCheckBox("Race several variants of each pass")
        auto = QCheckBox(text)
        spin = self._spin(True, minimum=2)
        spin.setValue(manual)

        def refresh():
            auto.setEnabled(race.isChecked())
            spin.setEnabled(race.isChecked() and not auto.isChecked())

        race.toggled.connect(refresh)
        auto.toggled.connect(refresh)
        form.addRow(race)
        row = QHBoxLayout()
        row.addWidget(auto, 1)
        row.addWidget(spin)
        form.addRow("How many", row)

        def write(value):
            race.setChecked(value != 1)
            auto.setChecked(value == 0)
            if value > 1:
                spin.setValue(value)
            refresh()

        self.readers["portfolio"] = lambda: 1 if not race.isChecked() else (0 if auto.isChecked() else spin.value())
        self.writers["portfolio"] = write

    def restore_defaults(self) -> None:
        self.load(Settings())

    def load(self, settings: Settings) -> None:
        for name, write in self.writers.items():
            write(getattr(settings, name))

    def settings(self) -> Settings:
        out = Settings()
        for name, read in self.readers.items():
            setattr(out, name, type(getattr(out, name))(read()))
        return out


class MainWindow(QMainWindow):
    def __init__(self, settings_path: str | None = None):
        super().__init__()
        self.setWindowTitle("WeaveEngine")
        self.resize(1400, 900)
        self.interactive = True  # False in the self-test: no dialogs that wait for a click
        self.settings_path = settings_path or default_path()
        self.settings = Settings.load(self.settings_path)
        self.dsn_path = None
        self.design = None       # as read from the file (shown before routing)
        self.routed_design = None  # as routed (rules applied), for export
        self.result = None
        self.want = None
        self.job = None
        self.started = 0.0
        self.tracker = None
        self.variants: dict[int, dict] = {}
        self.previous: dict[int, set] = {}
        self.shown_variant = None
        self.snapshots_seen = 0
        self.accel_status = None

        self.view = BoardView()
        self.setCentralWidget(self.view)
        self._build_actions()
        self._build_panel()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.pump)
        self.timer.start(40)
        # The compiled-kernel check can take about ten seconds the very first
        # time (it compiles): do it off the GUI thread and show the outcome.
        self.banner.setText("Checking the compiled kernels…")
        self.banner.setStyleSheet("background: #444; color: white; padding: 6px;")
        self.banner.show()
        threading.Thread(target=self._check_accel, daemon=True).start()

    # -- construction -----------------------------------------------------------
    def _build_actions(self) -> None:
        bar = self.addToolBar("Main")
        bar.setMovable(False)

        def action(text, slot, shortcut=None, enabled=True, tip=""):
            act = QAction(text, self)
            act.setToolTip(tip + (f"  ({shortcut})" if shortcut else ""))
            act.triggered.connect(slot)
            if shortcut:
                act.setShortcut(QKeySequence(shortcut))
            act.setEnabled(enabled)
            bar.addAction(act)
            return act

        self.act_open = action("Open DSN…", self.open_dialog, "Ctrl+O", tip="Open a Specctra DSN exported from your CAD tool")
        self.act_route = action("Route", self.route, "Ctrl+R", False, tip="Route the board")
        self.act_stop = action("Stop", self.stop, "Ctrl+.", False, tip="Stop routing")
        bar.addSeparator()
        self.act_ses = action("Export SES…", self.export_ses_dialog, "Ctrl+E", False, tip="Write the routed traces as a Specctra session to import into your CAD tool")
        self.act_svg = action("Export SVG…", self.export_svg_dialog, None, False, tip="Save a picture of the result")
        bar.addSeparator()
        self.act_settings = action("Settings…", self.edit_settings, "Ctrl+,", tip="Edit settings.json")
        self.act_fit = action("Zoom to Fit", self.view.fit, "Ctrl+0", tip="Zoom out so the whole board is visible (scroll to zoom, drag to pan)")

    def _build_panel(self) -> None:
        panel = QWidget()
        col = QVBoxLayout(panel)
        self.banner = QLabel()
        self.banner.setWordWrap(True)
        self.banner.hide()
        col.addWidget(self.banner)

        self.board_label = QLabel("No board loaded. Open a Specctra DSN file to begin.")
        self.board_label.setWordWrap(True)
        col.addWidget(self.board_label)

        self.pass_label = QLabel("")
        self.phase_label = QLabel("")
        self.progress = QProgressBar()
        self.progress.setRange(0, 1000)
        self.time_label = QLabel("")
        for w in (self.pass_label, self.progress, self.phase_label, self.time_label):
            col.addWidget(w)

        grid = QGridLayout()
        self.stats: dict[str, QLabel] = {}
        for row, (key, text) in enumerate([("routed", "Connections routed"), ("open", "Still open"), ("vias", "Vias"),
                                           ("rounds", "Rip-up rounds"), ("overflow", "Over-full gates"),
                                           ("length", "Trace length"), ("layers", "Wires per layer"),
                                           ("changed", "Just placed / rerouted"),
                                           ("showing", "Showing"), ("race", "Variants (open)")]):
            grid.addWidget(QLabel(text), row, 0)
            self.stats[key] = QLabel("–")
            self.stats[key].setWordWrap(True)
            grid.addWidget(self.stats[key], row, 1)
        col.addLayout(grid)

        self.layer_box = QHBoxLayout()
        col.addLayout(self.layer_box)

        show = QHBoxLayout()
        self.outline_box = QCheckBox("Outlines")
        self.outline_box.setToolTip("Draw the outline of every part")
        self.outline_box.setChecked(self.settings.show_outlines)
        self.outline_box.toggled.connect(self._toggle_outlines)
        self.name_box = QCheckBox("Names")
        self.name_box.setToolTip("Draw every part's reference")
        self.name_box.setChecked(self.settings.show_names)
        self.name_box.toggled.connect(self._toggle_names)
        show.addWidget(self.outline_box)
        show.addWidget(self.name_box)
        col.addLayout(show)
        self.view.show_outlines, self.view.show_names = self.settings.show_outlines, self.settings.show_names

        self.selection_label = QLabel("Click a trace, a pad or a part to see what it is.\nScroll to zoom, right-drag to pan.")
        self.selection_label.setWordWrap(True)
        self.selection_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.selection_label.setStyleSheet("background: #20242a; color: #e6e6e6; padding: 6px; border-radius: 4px;")
        col.addWidget(self.selection_label)
        self.view.selected.connect(self.show_selection)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(2000)
        col.addWidget(self.log, 1)

        dock = QDockWidget("Routing", self)
        dock.setWidget(panel)
        dock.setFeatures(QDockWidget.NoDockWidgetFeatures)
        dock.setMinimumWidth(340)
        self.addDockWidget(Qt.RightDockWidgetArea, dock)

    def say(self, text: str) -> None:
        self.log.appendPlainText(text)

    def _toggle_outlines(self, on: bool) -> None:
        self.view.set_outlines_visible(on)
        self._remember(show_outlines=on)

    def _toggle_names(self, on: bool) -> None:
        self.view.set_names_visible(on)
        self._remember(show_names=on)

    def _remember(self, **values) -> None:
        for name, value in values.items():
            setattr(self.settings, name, value)
        try:
            self.settings.save(self.settings_path)
        except OSError:
            pass  # a view preference that cannot be saved is not worth interrupting for

    def show_selection(self, info) -> None:
        if not info:
            self.selection_label.setText("Nothing there. Click a trace, a pad or a part.")
            return
        order = {"trace": ["net", "layer", "length", "width", "from", "to", "net total"],
                 "pad": ["name", "net", "part", "layers", "size"], "via": ["name", "net", "layers", "size"],
                 "part": ["name", "value", "footprint", "side", "position", "rotation", "pads"]}[info["kind"]]
        rows = "".join(f"<tr><td style='color:#9aa'>{key}&nbsp;&nbsp;</td><td>{info[key]}</td></tr>" for key in order if info.get(key))
        self.selection_label.setText(f"<b>{info['kind'].capitalize()}</b><table>{rows}</table>")

    # -- compiled kernels -----------------------------------------------------
    def _check_accel(self) -> None:
        self.accel_status = accel.check()  # never raises

    def _show_accel(self) -> None:
        status = self.accel_status
        if status.compiled:
            self.banner.setText(status.message + (f" (compiled in {status.seconds:.0f} s)" if status.seconds > 3 else ""))
            self.banner.setStyleSheet("background: #1f3d2a; color: #b7e4c7; padding: 6px;")
        else:
            self.banner.setText(status.warning.replace("WARNING: ", "⚠ "))
            self.banner.setStyleSheet("background: #7a1f1f; color: white; padding: 8px; font-weight: bold;")
            self.say(status.warning)
        self.banner.show()
        self._accel_shown = True

    # -- board ------------------------------------------------------------------
    def open_dialog(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Open a Specctra DSN", os.path.dirname(self.dsn_path or ""), "Specctra DSN (*.dsn);;All files (*)")
        if path:
            self.open(path)

    def open(self, path: str) -> bool:
        try:
            design = read_dsn(path)
        except (OSError, ValueError) as error:
            if self.interactive:
                QMessageBox.critical(self, "Cannot open the file", str(error))
            self.say(f"Cannot open {path}: {error}")
            return False
        self.stop()
        self.dsn_path, self.design, self.result, self.routed_design = path, design, None, None
        board = design.board
        self.view.set_board(board)
        nets = board.nets()
        connections = sum(len(p) - 1 for p in nets.values())
        self.board_label.setText(f"<b>{os.path.basename(path)}</b><br>{len(board.layers)} layers, {len(board.pads)} pads, "
                                 f"{len(nets)} nets, {connections} connections")
        self.say(f"Opened {path}")
        if board.plane_nets:
            self.say(f"  {len(board.plane_nets)} nets have a copper plane and will not be routed: "
                     + ", ".join(sorted(design.net_names[n] for n in board.plane_nets)))
        while self.layer_box.count():
            self.layer_box.takeAt(0).widget().deleteLater()
        for i, name in enumerate(board.layers):
            box = QCheckBox(name)
            box.setChecked(True)
            box.setStyleSheet(f"color: {LAYER_COLORS[i % len(LAYER_COLORS)]};")
            box.toggled.connect(lambda on, layer=i: self.view.set_layer_visible(layer, on))
            self.layer_box.addWidget(box)
        centre = {p.pad_id: p.centre for p in board.pads}
        from weaveengine.plan.context import spanning_pairs
        air = [(centre[a], centre[b]) for pads in nets.values() for a, b, _ in spanning_pairs(pads, centre)]
        self.view.show_routing([], air)
        for key in self.stats:
            self.stats[key].setText("–")
        self.stats["open"].setText(str(connections))
        self.progress.setValue(0)
        self.pass_label.setText("")
        self.phase_label.setText("")
        self.time_label.setText("")
        self.act_route.setEnabled(True)
        self.act_ses.setEnabled(False)
        self.act_svg.setEnabled(False)
        return True

    # -- routing ----------------------------------------------------------------
    def route(self) -> None:
        if self.dsn_path is None or (self.job is not None and self.job.running()):
            return
        status = self.accel_status
        if status is not None and not status.compiled and self.interactive:
            answer = QMessageBox.warning(self, "Compiled kernels unavailable", status.warning + "\n\nRoute anyway?",
                                         QMessageBox.Yes | QMessageBox.Cancel, QMessageBox.Cancel)
            if answer != QMessageBox.Yes:
                return
        self.result = None
        self.variants, self.shown_variant, self.previous = {}, None, {}
        self.snapshots_seen = 0
        self.tracker = ProgressBar(io.StringIO())
        self.started = time.time()
        self.job = Job(self.dsn_path, self.settings)
        self.job.start()
        self.say("Routing started")
        self.act_route.setEnabled(False)
        self.act_stop.setEnabled(True)
        self.act_open.setEnabled(False)
        self.act_settings.setEnabled(False)
        self.act_ses.setEnabled(False)
        self.act_svg.setEnabled(False)
        self.progress.setValue(0)
        self.phase_label.setText("starting…")

    def stop(self) -> None:
        if self.job is not None and self.job.running():
            self.job.cancel()
            self.say("Routing stopped")
            self._idle()

    def _idle(self) -> None:
        self.act_route.setEnabled(self.dsn_path is not None)
        self.act_stop.setEnabled(False)
        self.act_open.setEnabled(True)
        self.act_settings.setEnabled(True)

    def pump(self) -> None:
        """Timer tick: show the kernel check once it is in, and handle job events."""
        if self.accel_status is not None and not getattr(self, "_accel_shown", False):
            self._show_accel()
        if self.job is None:
            return
        draw = None
        for event in self.job.poll():
            kind = event["type"]
            if kind == "status":
                if not event["compiled"]:
                    self.say(event["warning"])
            elif kind == "note":
                self.say("  " + event["text"])
            elif kind == "board":
                self.routed_design = event["design"]
            elif kind == "progress":
                if event["variant"] <= 0:
                    self.tracker(f"pass {event['pass']}: {event['phase']}", event["done"], event["total"])
                    self.pass_label.setText(f"Pass {event['pass']}")
                    self.phase_label.setText(event["phase"])
            elif kind == "snapshot":
                self.snapshots_seen += 1
                if event["variant"] >= 0:
                    self.variants = {v: s for v, s in self.variants.items() if s["pass"] == event["pass"]}
                    self.variants[event["variant"]] = event
                    best = min(self.variants, key=lambda v: (self.variants[v]["total"] - self.variants[v]["routed"],
                                                             self.variants[v]["overflow"], v))
                    if best == event["variant"] or best != self.shown_variant:
                        draw = self.variants[best]
                else:
                    draw = event
            elif kind == "pass":
                self.say(f"Pass {event['pass']} finished: {event['open']} of {event['connections']} connections open, {event['vias']} vias")
                self.variants = {}
            elif kind == "done":
                self._finished(event)
                draw = None
            elif kind == "error":
                self.say("ERROR: " + event["text"])
                if event.get("trace"):
                    self.say(event["trace"])
                self.phase_label.setText("failed: " + event["text"])
                self._idle()
        if draw is not None:
            self._draw_snapshot(draw)
        if self.job.running():
            self.progress.setValue(int(1000 * self.tracker.fraction))
            left = self.tracker.remaining()
            self.time_label.setText(f"elapsed {clock(time.time() - self.started)}"
                                    + (f"   about {clock(left)} left in this pass" if left is not None else ""))

    def _draw_snapshot(self, snap: dict) -> None:
        # Wires that are new or have moved since the last picture of this variant.
        key = lambda w: (w[0], w[1], tuple(w[2][0]), tuple(w[2][-1]), len(w[2]))
        before = self.previous.get(snap["variant"])
        now = {key(w) for w in snap["wires"]}
        changed = [w for w in snap["wires"] if key(w) not in before] if before is not None and not snap["final"] else []
        self.previous = {snap["variant"]: now}
        self.shown_variant = snap["variant"]
        self.view.wire_info = {}
        self.view.highlight(None)
        self.view.show_routing(snap["wires"], snap["open"], snap["vias"], rough=not snap["final"], changed=changed)
        self.stats["changed"].setText(str(len(changed)) if changed else "–")
        self.stats["routed"].setText(f"{snap['routed']} / {snap['total']}")
        self.stats["open"].setText(str(snap["total"] - snap["routed"]))
        self.stats["vias"].setText(str(len(snap["vias"])))
        self.stats["rounds"].setText(str(snap["rounds"]))
        self.stats["overflow"].setText(str(snap["overflow"]))
        self.stats["length"].setText(f"{snap['length']:.0f} mm (estimate)")
        self.stats["layers"].setText(", ".join(f"{k}: {v}" for k, v in snap["per_layer"].items()))
        self.stats["showing"].setText("result of the pass" if snap["variant"] < 0 else
                                      f"variant {snap['variant'] + 1} (current best)" + ("" if snap["final"] else ", rough"))
        self.stats["race"].setText("  ".join(f"{v + 1}: {s['total'] - s['routed']}" for v, s in sorted(self.variants.items())) or "–")

    def _finished(self, event: dict) -> None:
        self.result = result = event["result"]
        self.routed_design = event["design"]
        self.want = event["want"]
        stats = result.stats
        teardrops = [(result.wire_layer[w], poly[:5]) for w, drops in result.teardrops.items() for poly in drops]
        wires = [(result.wire_layer[w], result.wire_net[w], pts) for w, pts in result.polylines.items()]
        centre = {p.pad_id: p.centre for p in result.board.pads}
        open_pairs = [(centre[c.src], centre[c.dst]) for c in result.connections.values()]
        pad_name = {p.pad_id: (p.name or str(p.pad_id)) for p in result.board.pads}
        self.view.wire_info = {i: {"from": pad_name[result.wire_pads[w][0]], "to": pad_name[result.wire_pads[w][1]]}
                               for i, w in enumerate(result.polylines) if w in result.wire_pads}
        self.view.show_routing(wires, open_pairs, result.vias, teardrops, result.violations)
        self.stats["routed"].setText(f"{stats['routed']} / {stats['connections']}")
        self.stats["open"].setText(str(stats["connections"] - stats["routed"]))
        self.stats["vias"].setText(str(stats["vias"]))
        self.stats["rounds"].setText(str(stats["ripup_rounds"]))
        self.stats["overflow"].setText("0")
        self.stats["length"].setText(f"{stats['length']:.0f} mm, {stats['length_ratio']:.2f} × straight-line")
        self.stats["layers"].setText(", ".join(f"{k}: {v}" for k, v in stats["wires_per_layer"].items()))
        self.stats["showing"].setText("final result" + (f", {len(teardrops)} teardrops" if teardrops else ""))
        self.stats["race"].setText("–")
        self.progress.setValue(1000)
        self.pass_label.setText("Finished")
        self.phase_label.setText(f"{stats['routed']} of {stats['connections']} connections routed")
        self.time_label.setText(f"took {clock(event['seconds'])}")
        self.say(f"Finished: {stats['routed']}/{stats['connections']} connections, {stats['vias']} vias, "
                 f"length ratio {stats['length_ratio']:.3f}, {event['seconds']:.1f} s")
        self._idle()
        self.act_ses.setEnabled(True)
        self.act_svg.setEnabled(True)

    # -- export -----------------------------------------------------------------
    def export_ses_dialog(self) -> None:
        if self.result is None:
            return
        suggested = os.path.splitext(self.dsn_path)[0] + ".ses"
        path, _ = QFileDialog.getSaveFileName(self, "Export the Specctra session", suggested, "Specctra session (*.ses)")
        if path:
            self.export_ses(path)

    def export_ses(self, path: str) -> bool:
        try:
            write_ses(self.routed_design, self.result, path, teardrops=self.settings.teardrops_in_ses)
            m = measure(self.routed_design, path, *self.want)
        except (OSError, ValueError) as error:
            self.say(f"Could not write {path}: {error}")
            if self.interactive:
                QMessageBox.critical(self, "Export failed", str(error))
            return False
        self.say(f"Wrote {path}")
        self.say(f"  measured from the file: smallest gap trace-pad {m.track_to_pad:.4f}, trace-trace {m.track_to_track:.4f} "
                 f"(rule {m.required_clearance:g}), trace-edge {m.track_to_edge:.4f} (rule {m.required_edge:g}) mm; "
                 f"{m.crossings} crossings -> {'OK' if m.ok else 'VIOLATIONS'}")
        return m.ok

    def export_svg_dialog(self) -> None:
        if self.result is None:
            return
        suggested = os.path.splitext(self.dsn_path)[0] + ".svg"
        path, _ = QFileDialog.getSaveFileName(self, "Export a picture", suggested, "SVG (*.svg)")
        if path:
            try:
                export_result(self.result, path)
                self.say(f"Wrote {path}")
            except OSError as error:
                QMessageBox.critical(self, "Export failed", str(error))

    # -- settings ---------------------------------------------------------------
    def edit_settings(self) -> None:
        dialog = SettingsDialog(self.settings, self.settings_path, self)
        if dialog.exec() == QDialog.Accepted:
            self.settings = dialog.settings()
            self.outline_box.setChecked(self.settings.show_outlines)
            self.name_box.setChecked(self.settings.show_names)
            try:
                self.settings.save(self.settings_path)
                self.say(f"Settings saved to {self.settings_path}")
            except OSError as error:
                QMessageBox.critical(self, "Could not save the settings", str(error))

    def closeEvent(self, event) -> None:
        if self.job is not None:
            self.job.cancel()
        super().closeEvent(event)


def self_test(dsn: str, ses: str | None, screenshot: str | None, timeout: float = 900.0) -> int:
    """Drives the window without a person: open, route, export, check. Used to
    test the application, including a packaged build:

        main.py --self-test board.dsn [--ses out.ses] [--screenshot out.png]
    """
    app = QApplication.instance() or QApplication(sys.argv[:1])
    window = MainWindow()
    window.interactive = False
    window.show()
    if not window.open(dsn):
        print("self-test: could not open", dsn)
        return 2
    deadline = time.time() + 120
    while window.accel_status is None and time.time() < deadline:
        app.processEvents()
        time.sleep(0.02)
    status = window.accel_status
    print("self-test: kernels:", status.message if status and status.compiled else (status.warning if status else "check timed out"))
    window.route()
    live_frames = 0
    live_shot = get("--live-shot") if (get := (lambda flag: sys.argv[sys.argv.index(flag) + 1] if flag in sys.argv else None)) else None
    live_after = float(get("--live-after") or 3.0)
    started = time.time()
    deadline = time.time() + timeout
    while window.result is None and window.job is not None and not window.job.finished and time.time() < deadline:
        before = window.shown_variant
        app.processEvents()
        live_frames += window.view.last is not None and window.shown_variant is not None and before is None
        if live_shot and window.shown_variant is not None and time.time() - started > live_after:
            window.grab().save(live_shot)
            print("self-test: live screenshot", live_shot, "at", round(time.time() - started, 1), "s")
            live_shot = None
        time.sleep(0.01)
    for _ in range(20):
        app.processEvents()
        time.sleep(0.01)
    if window.result is None:
        print("self-test: routing did not finish:", window.phase_label.text())
        print(window.log.toPlainText()[-2000:])
        window.close()
        return 3
    stats = window.result.stats
    print(f"self-test: routed {stats['routed']}/{stats['connections']}, vias {stats['vias']}, "
          f"teardrops {sum(len(d) for d in window.result.teardrops.values())}, "
          f"live picture shown: {bool(live_frames or window.snapshots_seen)} ({window.snapshots_seen} snapshots)")
    ok = stats["routed"] == stats["connections"]
    if ses:
        ok = window.export_ses(ses) and ok
        print("self-test:", window.log.toPlainText().splitlines()[-1].strip())
    view = window.view
    board = window.result.board
    # zoom about a point keeps that point still; many small steps do not run away
    probe = QPoint(view.viewport().width() // 3, view.viewport().height() // 3)
    anchor = view.mapToScene(probe)
    start = view.transform().m11()
    for _ in range(30):
        view.zoom_at(probe, math.exp(view.ZOOM_PER_PIXEL * 4))   # thirty small trackpad steps
    moved = view.mapToScene(probe)
    drift = math.hypot(moved.x() - anchor.x(), moved.y() - anchor.y()) * view.transform().m11()
    print(f"self-test: zoom x{view.transform().m11() / start:.2f} after 30 small steps, point under cursor drifted {drift:.2f} px")
    for _ in range(400):
        view.zoom_at(probe, 1.5)
    top = view.transform().m11()
    for _ in range(400):
        view.zoom_at(probe, 1 / 1.5)
    print(f"self-test: zoom limits {view.transform().m11() / start:.2f}x .. {top / start:.0f}x of fit")
    view.fit()
    before = view.mapToScene(probe)
    view.pan_by(40, -25)
    after = view.mapToScene(QPoint(probe.x() + 40, probe.y() - 25))
    print(f"self-test: pan error {math.hypot(after.x() - before.x(), after.y() - before.y()) * view.transform().m11():.2f} px")
    view.pan_by(-40, 25)
    # selection: a pad, a trace away from pads, a part's outline
    pad = next(p for p in board.pads if p.net_id >= 0)
    picked = [view.select_at(QPointF(pad.centre[0], -pad.centre[1]))]
    wid, line = max(window.result.polylines.items(), key=lambda kv: len(kv[1]))
    mid = line[len(line) // 2]
    picked.append(view.select_at(QPointF(mid[0], -mid[1])))
    part = next((c for c in board.components if c.bounds()), None)
    if part is not None:
        box = part.bounds()
        spot = next(((x, y) for x in (box[0] + 0.05, box[2] - 0.05) for y in (box[1] + 0.05, box[3] - 0.05)
                     if view.pick(x, y) and view.pick(x, y)["kind"] == "part"), None)
        picked.append(view.select_at(QPointF(spot[0], -spot[1])) if spot else None)
    for info in picked:
        print("self-test: selected", {k: v for k, v in (info or {}).items() if k not in ("outline", "trace", "pad", "layer index")})
    app.processEvents()
    if screenshot:
        window.grab().save(screenshot)
        print("self-test: screenshot", screenshot)
        view.zoom_at(view.mapFromScene(QPointF(mid[0], -mid[1])), 6.0)
        app.processEvents()
        window.grab().save(os.path.splitext(screenshot)[0] + "_zoom.png")
        dialog = SettingsDialog(window.settings, window.settings_path, window)
        dialog.show()
        tabs = dialog.findChild(QTabWidget)
        for i in range(tabs.count()):
            tabs.setCurrentIndex(i)
            app.processEvents()
            dialog.grab().save(os.path.splitext(screenshot)[0] + f"_settings{i + 1}.png")
        assert dialog.settings() == window.settings
        # every state the file can hold comes back out of the editor unchanged
        for odd in (Settings(workers=3, portfolio=1, trace_width=0.127, edge_clearance=0.5, teardrops=False),
                    Settings(workers=0, portfolio=5, via_diameter=0.45, via_drill=0.2, ignore_keepouts=True)):
            dialog.load(odd)
            assert dialog.settings() == odd, (dialog.settings(), odd)
        dialog.close()
        print("self-test: settings file", window.settings_path)
    window.close()
    return 0 if ok else 1


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv if argv is None else argv)
    if "--self-test" in argv:
        i = argv.index("--self-test")
        get = lambda flag: argv[argv.index(flag) + 1] if flag in argv else None
        return self_test(argv[i + 1], get("--ses"), get("--screenshot"))
    app = QApplication(argv[:1])
    app.setApplicationName("WeaveEngine")
    window = MainWindow()
    window.show()
    files = [a for a in argv[1:] if not a.startswith("-")]
    if files:
        window.open(files[0])
    return app.exec()
