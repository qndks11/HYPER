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

    def draw_course(self, background=None, extent=None):
        if background is not None:
            # zorder 0 so the course always draws over the imagery.
            self.ax.imshow(background, extent=extent, origin="upper", zorder=0)
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
        lines += ["", "좌클릭  배치", "우클릭  삭제", "n/p     라벨 이동",
                  "tab     다음 미배치", "u       되돌리기", "s       저장", "q       종료"]
        self.panel_text.set_text("\n".join(lines))

        star = "*" if self.dirty else ""
        self.ax.set_title(f"[{self.order[self.active]}]{star}   {self.status}",
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
    parser.add_argument("--background", help="georeferenced image to draw under the course")
    parser.add_argument("--extent", nargs=4, type=float,
                        metavar=("X0", "Y0", "X1", "Y1"),
                        help="--background's map-frame bounds in meters")
    parser.add_argument("--background-max-px", type=int, default=2500,
                        help="downsample --background to at most this many pixels on "
                             "its long edge (default: 2500)")
    parser.add_argument("--snap-warn", type=float, default=2.0,
                        help="warn when a click lands this far from any waypoint "
                             "(default: 2.0 m)")
    args = parser.parse_args()

    if args.background and not args.extent:
        sys.exit("--background requires --extent X0 Y0 X1 Y1.")
    if args.gazebo_course and args.background:
        sys.exit("--gazebo-course and --background are mutually exclusive.")

    setup_font()
    xs, ys = load_waypoints(args.csv_path)
    text, order, positions = load_mission(args.mission)

    background = extent = None
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
    elif args.background:
        background = load_background(args.background, args.background_max_px)
        x0, y0, x1, y1 = args.extent
        extent = [x0, x1, y0, y1]

    print(f"{len(xs)} waypoints from '{args.csv_path}'")
    print(f"mission '{args.mission}': {len(order)} label(s) required, "
          f"{len(positions)} already placed")

    app = Labeler(xs, ys, args.mission, text, order, positions, args.snap_warn)
    app.draw_course(background, extent)
    app.connect()
    app.refresh()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
