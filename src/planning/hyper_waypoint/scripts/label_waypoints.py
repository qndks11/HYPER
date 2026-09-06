#!/usr/bin/env python3
"""Interactively place mission event labels onto a recorded waypoint course.

Loads a waypoint_recorder_node CSV and hyper_planner's mission.yaml, draws the course,
and lets you click each event point (stop lines, traffic lights, parking bays). Every
click snaps to the nearest recorded waypoint, so a label always lands on a pose the
vehicle actually drove through.

Labels are stored as map-frame coordinates rather than waypoint indices: re-recording
the course renumbers every index, but the physical stop line does not move, so
coordinate labels survive a re-record while index labels would silently shift.

The label list is not hardcoded here -- it is whatever the mission's `drive` steps
reference via `until:`, so adding a step to mission.yaml is enough to make this tool
ask for its position.

Only the `labels:` block of mission.yaml is rewritten on save; the rest of the file
(comments, steps, tuning parameters) is spliced back byte-for-byte, because PyYAML
round-tripping would drop every comment in the file.

Usage:
    python3 label_waypoints.py waypoints/sim.csv
    python3 label_waypoints.py waypoints/track.csv --mission /path/to/mission.yaml
    python3 label_waypoints.py waypoints/sim.csv --gazebo-course
    python3 label_waypoints.py waypoints/full_track.csv --real-course
    python3 label_waypoints.py waypoints/track.csv --background ortho.png \
        --extent -50 -60 60 50
"""
import argparse
import csv
import os
import re
import sys

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import yaml

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# The Gazebo course's own ground texture, which --gazebo-course draws under the
# waypoints. Its map-frame bounds are not hardcoded: they are read from the quad in
# ground.obj, so regenerating the mesh (build_course.py) cannot silently desync the
# overlay from what the simulator actually renders.
GAZEBO_COURSE_MESHES = os.path.normpath(os.path.join(
    SCRIPT_DIR, "..", "..", "..", "simulator", "hyper_gazebo", "worlds", "models",
    "driving_course", "meshes"))

# The real course's aerial screenshot. Unlike the Gazebo texture it carries no
# georeference at all, so its map-frame placement is whatever the operator aligned it
# to, stored beside the image in <image>.align.yaml (see load_alignment).
REAL_COURSE_IMAGE = os.path.join(GAZEBO_COURSE_MESHES, "real_course.png")

DEFAULT_MISSION = os.path.join(
    SCRIPT_DIR, "..", "..", "hyper_planner", "config", "mission.yaml")

# The block this tool owns and regenerates wholesale.
LABELS_HEADER = (
    "labels:\n"
    "  # 이 블록은 label_waypoints.py가 통째로 재작성합니다."
    " 손으로 쓴 주석을 여기 두지 마세요.\n")

PAGE = 9  # number keys 1..9 select within the current page

# matplotlib's default DejaVu Sans has no Hangul, so the status line and side panel
# would render as tofu boxes. Prefer whichever Korean face the machine actually has.
# "Noto Sans CJK JP" is in the list on purpose: the pan-CJK Noto faces carry the full
# Hangul range regardless of which language the family is named after.
KOREAN_FONTS = ("NanumGothic", "NanumBarunGothic", "Noto Sans CJK KR",
                "Noto Sans KR", "Noto Sans CJK JP", "Malgun Gothic")
KOREAN_FONT_FILES = ("NanumGothic.ttf", "NanumBarunGothic.ttf",
                     "NotoSansCJK-Regular.ttc", "NotoSansCJKkr-Regular.otf")
# The side panel is monospaced for column alignment, and font.family does not apply to
# it -- font.monospace does, and its defaults are all Hangul-less. Needs its own face.
KOREAN_MONO_FONTS = ("NanumGothicCoding", "D2Coding", "Noto Sans Mono CJK KR",
                     "Noto Sans Mono CJK JP")
KOREAN_MONO_FONT_FILES = ("NanumGothicCoding.ttf", "D2Coding.ttf",
                          "NotoSansMonoCJK-Regular.ttc")


def _resolve(names, basenames):
    """Return an installed font's family name, registering it from disk when
    matplotlib's cached font list predates its installation (the cache is built once
    and never notices fonts added later -- exactly the case on this machine)."""
    from matplotlib import font_manager
    available = {f.name for f in font_manager.fontManager.ttflist}
    for name in names:
        if name in available:
            return name
    on_disk = {os.path.basename(path): path
               for path in font_manager.findSystemFonts(fontext="ttf")}
    for basename in basenames:
        if basename in on_disk:
            font_manager.fontManager.addfont(on_disk[basename])
            return font_manager.FontProperties(fname=on_disk[basename]).get_name()
    return None


def setup_font():
    body = _resolve(KOREAN_FONTS, KOREAN_FONT_FILES)
    if body is None:
        print("경고: 한글 폰트를 찾지 못했습니다. 화면의 한글이 깨져 보일 수 있습니다 "
              "(sudo apt install fonts-nanum).", file=sys.stderr)
        return None, None

    plt.rcParams["font.family"] = body
    # Korean faces ship no U+2212, so negative tick labels (this course has plenty of
    # them) would tofu too; fall back to the ASCII hyphen-minus.
    plt.rcParams["axes.unicode_minus"] = False

    # Fall back to the proportional face if no Korean monospace exists: the panel's
    # columns drift, but the text stays readable, which matters more.
    mono = _resolve(KOREAN_MONO_FONTS, KOREAN_MONO_FONT_FILES) or body
    plt.rcParams["font.monospace"] = [mono] + list(plt.rcParams["font.monospace"])
    return body, mono


def load_waypoints(csv_path):
    """Return (xs, ys) of every row with a usable x/y, matching plot_waypoints.py's
    header-name parsing so both the minimal and the full recorder layout work."""
    xs, ys = [], []
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None or "x" not in reader.fieldnames:
            sys.exit(f"'{csv_path}' has no 'x'/'y' header columns.")
        for row in reader:
            if not row.get("x") or not row.get("y"):
                continue
            try:
                xs.append(float(row["x"]))
                ys.append(float(row["y"]))
            except ValueError:
                continue
    if not xs:
        sys.exit(f"'{csv_path}' contains no usable waypoints.")
    return np.array(xs), np.array(ys)


def read_obj_extent(obj_path):
    """Return the [x0, x1, y0, y1] bounds of a textured ground quad's vertices.

    ground.obj maps the texture corner-to-corner (vt 0..1 across the quad), so the
    vertex bounds are exactly the image's map-frame footprint. Note the texture's pixel
    aspect does not match the quad's -- Gazebo stretches it to fit, and reusing these
    bounds reproduces that same stretch, which is what makes recorded waypoints line up.
    """
    xs, ys = [], []
    with open(obj_path) as f:
        for line in f:
            if line.startswith("v "):
                parts = line.split()
                xs.append(float(parts[1]))
                ys.append(float(parts[2]))
    if not xs:
        sys.exit(f"'{obj_path}' has no vertices.")
    return [min(xs), max(xs), min(ys), max(ys)]


def load_background(path, max_px):
    """Load an image for the backdrop, downsampled so panning stays responsive.

    The Gazebo course texture is 3937 x 4492; matplotlib's own imread would expand that
    to a ~280 MB float32 array and make every redraw crawl. Downsampled to max_px the
    resolution is still far finer than the ~10 cm placement precision this tool needs.
    """
    try:
        from PIL import Image
    except ImportError:
        return plt.imread(path)

    image = Image.open(path).convert("RGB")
    scale = min(1.0, max_px / max(image.size))
    if scale < 1.0:
        new_size = (max(1, round(image.width * scale)),
                    max(1, round(image.height * scale)))
        image = image.resize(new_size, Image.BILINEAR)
    return np.asarray(image)


def alignment_path(image_path):
    """Sidecar holding an image's map-frame placement, e.g. real_course.align.yaml."""
    return os.path.splitext(image_path)[0] + ".align.yaml"


def load_alignment(image_path, xs, ys):
    """Return the placement of `image_path` as (cx, cy, width_m, rot_deg).

    An aerial screenshot of the real course has no georeference of any kind, so the
    placement lives in a sidecar the operator writes from inside this tool ('w' in
    alignment mode). Until that file exists, start from a guess that at least puts the
    image on screen with the waypoints: centred on the course, a little wider than the
    driven extent, unrotated.

    The scale is stored as the image's full width in meters rather than a
    meters-per-pixel figure, because load_background downsamples: a pixel-based scale
    would silently shift the overlay whenever --background-max-px changed.
    """
    path = alignment_path(image_path)
    if os.path.exists(path):
        try:
            with open(path) as f:
                doc = yaml.safe_load(f) or {}
            return (float(doc["center_x"]), float(doc["center_y"]),
                    float(doc["width_m"]), float(doc.get("rotation_deg", 0.0)))
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
            print(f"경고: '{path}'를 읽지 못해 초기 정렬값을 추정합니다 ({exc}).",
                  file=sys.stderr)

    return (float((xs.min() + xs.max()) / 2.0), float((ys.min() + ys.max()) / 2.0),
            1.4 * max(float(xs.max() - xs.min()), 1.0), 0.0)


def save_alignment(image_path, cx, cy, width_m, rot_deg):
    """Write the placement sidecar. Raises OSError, which the caller reports."""
    path = alignment_path(image_path)
    with open(path, "w") as f:
        f.write("# label_waypoints.py의 정렬 모드(a)가 저장한 배경 이미지 위치입니다.\n")
        f.write(f"# image: {os.path.basename(image_path)}\n")
        f.write("# 이미지 중심의 map 좌표(m), 이미지 가로 폭(m), 반시계 회전각(도).\n")
        f.write(f"center_x: {cx:.4f}\n")
        f.write(f"center_y: {cy:.4f}\n")
        f.write(f"width_m: {width_m:.4f}\n")
        f.write(f"rotation_deg: {rot_deg:.4f}\n")
    return path


def load_mission(path):
    """Return (raw_text, required_labels_in_step_order, existing_positions)."""
    try:
        with open(path) as f:
            text = f.read()
    except OSError as exc:
        sys.exit(f"Failed to read mission file: {exc}")

    try:
        doc = yaml.safe_load(text) or {}
    except yaml.YAMLError as exc:
        sys.exit(f"'{path}' is not valid YAML: {exc}")

    required = []
    for step in doc.get("steps") or []:
        if isinstance(step, dict) and step.get("until") not in (None, ""):
            if step["until"] not in required:
                required.append(step["until"])
    if not required:
        sys.exit(f"'{path}' has no steps with an 'until:' label to place.")

    positions = {}
    for name, value in (doc.get("labels") or {}).items():
        if isinstance(value, dict) and "x" in value and "y" in value:
            positions[name] = (float(value["x"]), float(value["y"]))
    return text, required, positions


def render_labels_block(positions, indices, order):
    """Serialise the labels block, orphans (not referenced by any step) last so they
    stay visible for deletion instead of being silently dropped."""
    lines = [LABELS_HEADER]
    orphans = [n for n in positions if n not in order]
    for name in list(order) + sorted(orphans):
        if name not in positions:
            continue
        x, y = positions[name]
        note = f"   # wp #{indices[name]}" if name in indices else ""
        tag = "  (orphan: 어떤 step도 참조하지 않음)" if name in orphans else ""
        lines.append(f"  {name}: {{x: {x:.3f}, y: {y:.3f}}}{note}{tag}\n")
    return "".join(lines)


def splice_labels(text, block):
    """Replace the top-level `labels:` mapping in `text` with `block`, leaving every
    other byte of the file untouched."""
    lines = text.splitlines(keepends=True)
    start = None
    for i, line in enumerate(lines):
        if re.match(r"^labels:\s*(#.*)?$", line):
            start = i
            break

    if start is None:
        # No labels key yet: insert just above `steps:`, else append.
        for i, line in enumerate(lines):
            if re.match(r"^steps:\s*(#.*)?$", line):
                return "".join(lines[:i]) + block + "\n" + "".join(lines[i:])
        return text + ("" if text.endswith("\n") else "\n") + block

    # The block runs until the next line that starts at column 0 (a sibling key or a
    # comment introducing one); trailing blank lines belong to the separator, not to us.
    end = start + 1
    while end < len(lines) and (lines[end].strip() == "" or lines[end][:1] in " \t"):
        end += 1
    while end > start + 1 and lines[end - 1].strip() == "":
        end -= 1
    return "".join(lines[:start]) + block + "".join(lines[end:])


class Labeler:
    def __init__(self, xs, ys, mission_path, text, order, positions, snap_warn):
        self.xs, self.ys = xs, ys
        self.mission_path = mission_path
        self.text = text
        self.order = order
        self.positions = dict(positions)
        self.indices = {}
        self.snap_warn = snap_warn
        self.active = 0
        self.page = 0
        self.dirty = False
        self.status = "클릭해서 라벨을 배치하세요."
        self.undo_stack = []

        # Any label loaded from the file already has a position; recover its waypoint
        # index so the regenerated block keeps its `# wp #N` note.
        for name, (x, y) in self.positions.items():
            self.indices[name] = int(np.argmin(np.hypot(xs - x, ys - y)))

        # matplotlib binds s/p/q/g/l/k/o/f to figure actions by default, which would
        # fire alongside our own handlers. Release the ones we use.
        for key in ("save", "pan", "quit", "grid", "grid_minor", "yscale", "xscale",
                    "zoom", "home", "back", "forward", "fullscreen"):
            plt.rcParams[f"keymap.{key}"] = []

        self.fig, (self.ax, self.panel) = plt.subplots(
            1, 2, figsize=(15, 9), gridspec_kw={"width_ratios": [4, 1]})
        self.fig.canvas.manager.set_window_title(f"label_waypoints -- {mission_path}")
        self.panel.axis("off")
        self.panel_text = self.panel.text(
            0.0, 1.0, "", va="top", ha="left", family="monospace", fontsize=9,
            transform=self.panel.transAxes)

        self.marks = None
        self.annotations = []

        # Background alignment state, only meaningful for an image placed by hand
        # (--real-course / --background --align): the artist plus its map-frame
        # placement, and whether the keyboard is currently driving it.
        self.bg_artist = None
        self.align_image = None
        self.align_cx = self.align_cy = 0.0
        self.align_w = 1.0
        self.align_rot = 0.0
        self.align_mode = False
        self.align_dirty = False
        self.bg_alpha = 1.0

    def draw_course(self, background=None, extent=None, align=None):
        if background is not None and align is not None:
            self.align_image = align["path"]
            self.align_cx, self.align_cy = align["cx"], align["cy"]
            self.align_w, self.align_rot = align["width_m"], align["rot_deg"]
            # Drawn in a unit-width image frame and placed by an affine transform, so
            # scale/rotation stay live: the pixel array never has to be re-sampled.
            aspect = background.shape[0] / background.shape[1]
            self.bg_artist = self.ax.imshow(
                background, origin="upper", zorder=0, alpha=self.bg_alpha,
                extent=[-0.5, 0.5, -0.5 * aspect, 0.5 * aspect])
            self.apply_alignment()
            self.frame_view()
        elif background is not None:
            # zorder 0 so the course always draws over the imagery.
            self.bg_artist = self.ax.imshow(background, extent=extent,
                                            origin="upper", zorder=0)
        # Grey reads well on a blank figure but disappears into asphalt; cyan holds up
        # against both the road and the grass in the course texture, and stays clear of
        # the blue/red the label markers use.
        self.ax.plot(self.xs, self.ys, ".", ms=2,
                     color="#00e5ff" if background is not None else "0.55",
                     zorder=1, label="waypoints")
        self.ax.plot(self.xs[0], self.ys[0], "o", ms=10, mfc="none", mec="tab:green",
                     mew=2, zorder=3, label="start")
        self.ax.plot(self.xs[-1], self.ys[-1], "s", ms=10, mfc="none", mec="tab:red",
                     mew=2, zorder=3, label="end")
        self.ax.set_aspect("equal")
        self.ax.set_xlabel("x [m]  (map frame)")
        self.ax.set_ylabel("y [m]")
        self.ax.grid(alpha=0.3)
        self.ax.legend(loc="upper right", fontsize=8)

    # -- background alignment ---------------------------------------------------
    def apply_alignment(self):
        """Push the current placement onto the background artist."""
        from matplotlib.transforms import Affine2D
        transform = (Affine2D().scale(self.align_w).rotate_deg(self.align_rot)
                     .translate(self.align_cx, self.align_cy) + self.ax.transData)
        self.bg_artist.set_transform(transform)

    def frame_view(self):
        """Set the axes limits by hand around waypoints and image alike.

        A transformed AxesImage contributes nothing to matplotlib's autoscaling, so
        without this a badly-placed overlay would simply be off-screen with no hint
        that it exists.
        """
        corners = np.array([[-0.5, -0.5, 0.5, 0.5], [-0.5, 0.5, 0.5, -0.5]])
        aspect = self.bg_artist.get_extent()
        corners[1] *= (aspect[3] - aspect[2]) / (aspect[1] - aspect[0])
        angle = np.radians(self.align_rot)
        rot = np.array([[np.cos(angle), -np.sin(angle)],
                        [np.sin(angle), np.cos(angle)]])
        pts = rot @ (corners * self.align_w)
        bx = pts[0] + self.align_cx
        by = pts[1] + self.align_cy
        pad = 5.0
        self.ax.set_xlim(min(bx.min(), self.xs.min()) - pad,
                         max(bx.max(), self.xs.max()) + pad)
        self.ax.set_ylim(min(by.min(), self.ys.min()) - pad,
                         max(by.max(), self.ys.max()) + pad)

    def nudge(self, dx=0.0, dy=0.0, scale=1.0, rot=0.0, alpha=0.0):
        self.align_cx += dx
        self.align_cy += dy
        self.align_w *= scale
        self.align_rot += rot
        if alpha:
            self.bg_alpha = float(np.clip(self.bg_alpha + alpha, 0.1, 1.0))
            self.bg_artist.set_alpha(self.bg_alpha)
        self.align_dirty = True
        self.apply_alignment()
        self.status = (f"정렬: 중심 ({self.align_cx:.2f}, {self.align_cy:.2f}) "
                       f"폭 {self.align_w:.2f} m  회전 {self.align_rot:.2f}°  "
                       f"불투명도 {self.bg_alpha:.1f}")

    def save_align(self):
        try:
            path = save_alignment(self.align_image, self.align_cx, self.align_cy,
                                  self.align_w, self.align_rot)
        except OSError as exc:
            self.status = f"정렬 저장 실패: {exc}"
            return
        self.align_dirty = False
        self.status = f"{os.path.basename(path)} 저장됨"
        print(f"{path} 저장됨")

    def align_keys(self, key):
        """Handle one key in alignment mode. Returns False if it was not ours."""
        step = 0.2 if key.startswith("shift+") else 2.0
        bare = key[len("shift+"):] if key.startswith("shift+") else key
        fine = key.startswith("shift+")
        if bare == "left":
            self.nudge(dx=-step)
        elif bare == "right":
            self.nudge(dx=step)
        elif bare == "up":
            self.nudge(dy=step)
        elif bare == "down":
            self.nudge(dy=-step)
        elif key in ("+", "=", "shift+="):
            self.nudge(scale=1.001 if fine else 1.01)
        elif key in ("-", "_", "shift+-"):
            self.nudge(scale=1 / (1.001 if fine else 1.01))
        elif bare == ",":
            self.nudge(rot=0.05 if fine else 0.5)
        elif bare == ".":
            self.nudge(rot=-0.05 if fine else -0.5)
        elif bare == "v":
            self.nudge(alpha=-0.1)
        elif bare == "b":
            self.nudge(alpha=0.1)
        elif bare == "f":
            self.frame_view()
            self.status = "화면을 코스에 맞췄습니다."
        elif bare == "w":
            self.save_align()
        else:
            return False
        return True

    def connect(self):
        self.fig.canvas.mpl_connect("button_press_event", self.on_click)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        self.fig.canvas.mpl_connect("close_event", self.on_close)

    # -- interaction ------------------------------------------------------------
    def navigating(self):
        """True while the toolbar's pan/zoom tool is armed. Without this guard every
        drag to zoom into a stop line would also drop a label."""
        toolbar = getattr(self.fig.canvas, "toolbar", None)
        return bool(getattr(toolbar, "mode", ""))

    def on_click(self, event):
        if event.inaxes is not self.ax or event.xdata is None or self.navigating():
            return
        if self.align_mode:
            if event.button == 1:
                self.nudge(dx=event.xdata - self.align_cx,
                           dy=event.ydata - self.align_cy)
                self.refresh()
            return
        if event.button == 1:
            self.place(event.xdata, event.ydata)
        elif event.button == 3:
            self.delete_near(event.xdata, event.ydata)
        self.refresh()

    def place(self, cx, cy):
        name = self.order[self.active]
        i = int(np.argmin(np.hypot(self.xs - cx, self.ys - cy)))
        dist = float(np.hypot(self.xs[i] - cx, self.ys[i] - cy))
        self.undo_stack.append((name, self.positions.get(name), self.indices.get(name)))
        self.positions[name] = (float(self.xs[i]), float(self.ys[i]))
        self.indices[name] = i
        self.dirty = True
        warn = "  ** 경로에서 먼 클릭입니다 **" if dist > self.snap_warn else ""
        self.status = (f"{name} -> wp #{i} ({self.xs[i]:.2f}, {self.ys[i]:.2f}), "
                       f"스냅 {dist:.2f} m{warn}")
        self.advance_to_unplaced()

    def delete_near(self, cx, cy):
        if not self.positions:
            self.status = "삭제할 라벨이 없습니다."
            return
        name = min(self.positions,
                   key=lambda n: np.hypot(self.positions[n][0] - cx,
                                          self.positions[n][1] - cy))
        self.undo_stack.append((name, self.positions[name], self.indices.get(name)))
        del self.positions[name]
        self.indices.pop(name, None)
        self.dirty = True
        self.status = f"{name} 삭제됨."

    def undo(self):
        if not self.undo_stack:
            self.status = "되돌릴 작업이 없습니다."
            return
        name, position, index = self.undo_stack.pop()
        if position is None:
            self.positions.pop(name, None)
            self.indices.pop(name, None)
        else:
            self.positions[name] = position
            self.indices[name] = index
        self.dirty = True
        self.status = f"되돌림: {name}"

    def advance_to_unplaced(self):
        for offset in range(1, len(self.order) + 1):
            i = (self.active + offset) % len(self.order)
            if self.order[i] not in self.positions:
                self.active = i
                self.page = i // PAGE
                return

    def on_key(self, event):
        key = event.key
        if key == "a":
            if self.align_image is None:
                self.status = "정렬할 배경 이미지가 없습니다 (--real-course)."
            else:
                self.align_mode = not self.align_mode
                self.status = ("정렬 모드: 방향키 이동, +/- 축척, ,/. 회전, w 저장"
                               if self.align_mode else "정렬 모드 종료.")
            self.refresh()
            return
        if self.align_mode and key and self.align_keys(key):
            self.refresh()
            return
        if key in ("n", "right"):
            self.active = (self.active + 1) % len(self.order)
            self.page = self.active // PAGE
        elif key in ("p", "left"):
            self.active = (self.active - 1) % len(self.order)
            self.page = self.active // PAGE
        elif key == "tab":
            self.advance_to_unplaced()
        elif key in ("]", "["):
            pages = (len(self.order) + PAGE - 1) // PAGE
            self.page = (self.page + (1 if key == "]" else -1)) % pages
        elif key and key.isdigit() and key != "0":
            i = self.page * PAGE + int(key) - 1
            if i < len(self.order):
                self.active = i
            else:
                self.status = f"{key}번은 이 페이지에 없습니다."
        elif key == "u":
            self.undo()
        elif key == "s":
            self.save()
        elif key == "q":
            plt.close(self.fig)
            return
        else:
            return
        self.refresh()

    def on_close(self, _event):
        if self.dirty:
            print("\n경고: 저장하지 않은 변경이 있습니다 (저장은 's' 키).", file=sys.stderr)
        if self.align_dirty:
            print("경고: 저장하지 않은 배경 정렬이 있습니다 (저장은 정렬 모드의 'w' 키).",
                  file=sys.stderr)

    # -- persistence ------------------------------------------------------------
    def save(self):
        missing = [n for n in self.order if n not in self.positions]
        block = render_labels_block(self.positions, self.indices, self.order)
        try:
            updated = splice_labels(self.text, block)
            with open(self.mission_path, "w") as f:
                f.write(updated)
        except OSError as exc:
            self.status = f"저장 실패: {exc}"
            return
        self.text = updated
        self.dirty = False
        note = f" (미배치 {len(missing)}개: {', '.join(missing)})" if missing else ""
        # Basename only: the title bar already carries the full path, and a long one
        # pushes the status message off the figure.
        self.status = f"{os.path.basename(self.mission_path)} 저장됨{note}"
        print(f"{self.mission_path} 저장됨{note}")

    # -- rendering --------------------------------------------------------------
    def refresh(self):
        for artist in self.annotations:
            artist.remove()
        self.annotations = []

        for name, (x, y) in self.positions.items():
            is_active = name == self.order[self.active]
            color = "tab:red" if is_active else "tab:blue"
            self.annotations.append(
                self.ax.plot(x, y, "X", ms=12, color=color, zorder=5)[0])
            self.annotations.append(
                self.ax.annotate(
                    name, (x, y), textcoords="offset points", xytext=(8, 8),
                    fontsize=9, color=color, zorder=6,
                    bbox=dict(boxstyle="round,pad=0.2", fc="white", ec=color,
                              alpha=0.85)))

        placed = len(self.positions)
        pages = (len(self.order) + PAGE - 1) // PAGE
        lines = [f"배치 {placed}/{len(self.order)}",
                 f"페이지 {self.page + 1}/{pages}  ([ ])", ""]
        for slot in range(PAGE):
            i = self.page * PAGE + slot
            if i >= len(self.order):
                break
            name = self.order[i]
            mark = "O" if name in self.positions else "."
            cursor = ">" if i == self.active else " "
            lines.append(f"{cursor}{slot + 1}. [{mark}] {name}")
        orphans = [n for n in self.positions if n not in self.order]
        if orphans:
            lines += ["", "orphan (step 미참조):"] + [f"   {n}" for n in orphans]
        if self.align_mode:
            lines = ["[정렬 모드]  a 로 종료", "",
                     f"중심  ({self.align_cx:.2f}, {self.align_cy:.2f})",
                     f"폭    {self.align_w:.2f} m",
                     f"회전  {self.align_rot:.2f}°",
                     f"투명  {self.bg_alpha:.1f}", "",
                     "방향키  이동 (2 m)", "shift+  미세 (0.2 m)",
                     "+/-     축척 ±1%", ",/.     회전 ±0.5°",
                     "좌클릭  이미지 중심 이동", "v/b     투명도",
                     "f       화면 맞춤", "w       정렬 저장", "a       정렬 모드 종료"]
        else:
            lines += ["", "좌클릭  배치", "우클릭  삭제", "n/p     라벨 이동",
                      "tab     다음 미배치", "u       되돌리기", "s       저장",
                      "q       종료"]
            if self.align_image is not None:
                lines.append("a       배경 정렬 모드")
        self.panel_text.set_text("\n".join(lines))

        star = "*" if self.dirty else ""
        head = "[정렬]" if self.align_mode else f"[{self.order[self.active]}]"
        self.ax.set_title(f"{head}{star}   {self.status}",
                          fontsize=10, loc="left")
        self.fig.canvas.draw_idle()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv_path", help="waypoint_recorder_node output CSV")
    parser.add_argument("--mission", default=os.path.normpath(DEFAULT_MISSION),
                        help="mission.yaml to read and update (default: hyper_planner's)")
    parser.add_argument("--gazebo-course", action="store_true",
                        help="draw hyper_gazebo's driving_course texture as the "
                             "backdrop, with bounds read from its ground.obj "
                             "(simulation CSVs only)")
    parser.add_argument("--real-course", action="store_true",
                        help="draw the real course's aerial image "
                             f"({os.path.basename(REAL_COURSE_IMAGE)}) as the backdrop, "
                             "positioned by its .align.yaml sidecar and adjustable in "
                             "the tool's alignment mode ('a')")
    parser.add_argument("--real-course-image", default=REAL_COURSE_IMAGE,
                        help="image --real-course draws (default: hyper_gazebo's "
                             "driving_course/meshes/real_course.png)")
    parser.add_argument("--background", help="georeferenced image to draw under the course")
    parser.add_argument("--extent", nargs=4, type=float,
                        metavar=("X0", "Y0", "X1", "Y1"),
                        help="--background's map-frame bounds in meters")
    parser.add_argument("--background-max-px", type=int, default=2500,
                        help="downsample --background to at most this many pixels on "
                             "its long edge (default: 2500)")
    parser.add_argument("--align", action="store_true",
                        help="start in background alignment mode (implied use with "
                             "--real-course; also aligns --background, whose --extent "
                             "is then only the starting placement)")
    parser.add_argument("--snap-warn", type=float, default=2.0,
                        help="warn when a click lands this far from any waypoint "
                             "(default: 2.0 m)")
    args = parser.parse_args()

    backdrops = sum(bool(v) for v in (args.gazebo_course, args.background,
                                      args.real_course))
    if backdrops > 1:
        sys.exit("--gazebo-course, --real-course and --background are mutually "
                 "exclusive.")
    if args.background and not args.extent and not args.align:
        sys.exit("--background requires --extent X0 Y0 X1 Y1 (or --align).")
    if args.align and args.gazebo_course:
        sys.exit("--gazebo-course is georeferenced by ground.obj; nothing to align.")

    setup_font()
    xs, ys = load_waypoints(args.csv_path)
    text, order, positions = load_mission(args.mission)

    background = extent = align = None
    if args.gazebo_course:
        texture = os.path.join(GAZEBO_COURSE_MESHES, "course.png")
        quad = os.path.join(GAZEBO_COURSE_MESHES, "ground.obj")
        for required in (texture, quad):
            if not os.path.exists(required):
                sys.exit(f"--gazebo-course needs '{required}', which is missing.")
        extent = read_obj_extent(quad)
        background = load_background(texture, args.background_max_px)
        print(f"배경: {texture}  범위 x[{extent[0]:.2f}, {extent[1]:.2f}] "
              f"y[{extent[2]:.2f}, {extent[3]:.2f}] m")
    elif args.real_course or (args.background and args.align):
        image_path = args.background or args.real_course_image
        if not os.path.exists(image_path):
            sys.exit(f"배경 이미지 '{image_path}'가 없습니다.")
        background = load_background(image_path, args.background_max_px)
        cx, cy, width_m, rot = load_alignment(image_path, xs, ys)
        if args.extent and not os.path.exists(alignment_path(image_path)):
            # No sidecar yet, but the caller gave bounds: start from those instead of
            # the generic guess, so --extent stays useful as a rough first placement.
            x0, y0, x1, y1 = args.extent
            cx, cy, width_m, rot = (x0 + x1) / 2.0, (y0 + y1) / 2.0, abs(x1 - x0), 0.0
        align = {"path": image_path, "cx": cx, "cy": cy, "width_m": width_m,
                 "rot_deg": rot}
        sidecar = alignment_path(image_path)
        print(f"배경: {image_path}")
        print(f"정렬: {sidecar if os.path.exists(sidecar) else '(없음, 추정값 사용)'} "
              f"-- 중심 ({cx:.2f}, {cy:.2f}) 폭 {width_m:.2f} m 회전 {rot:.2f}°")
        print("'a'로 정렬 모드에 들어가 배경을 코스에 맞춘 뒤 'w'로 저장하세요.")
    elif args.background:
        background = load_background(args.background, args.background_max_px)
        x0, y0, x1, y1 = args.extent
        extent = [x0, x1, y0, y1]

    print(f"{len(xs)} waypoints from '{args.csv_path}'")
    print(f"mission '{args.mission}': {len(order)} label(s) required, "
          f"{len(positions)} already placed")

    app = Labeler(xs, ys, args.mission, text, order, positions, args.snap_warn)
    app.draw_course(background, extent, align)
    if align is not None and args.align:
        app.align_mode = True
        app.status = "정렬 모드: 방향키 이동, +/- 축척, ,/. 회전, w 저장"
    app.connect()
    app.refresh()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
