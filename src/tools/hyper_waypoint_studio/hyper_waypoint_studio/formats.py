#!/usr/bin/env python3
# =====================================================================
# 파일 입출력만 담당합니다 -- Qt도 ROS도 import하지 않습니다.
#
# 그렇게 나눈 이유는 두 가지입니다. (1) 보기/편집은 ROS 그래프 없이도 돌아야 하고,
# (2) 여기 있는 함수들이 곧 "저장이 원본을 안 깨뜨린다"의 근거라서 GUI 없이 그냥
# python3로 돌려 볼 수 있어야 합니다.
#
# 대부분은 label_waypoints.py에서 그대로 옮겨 온 것입니다. 옮기면서 고친 것은 없고,
# 왜 그렇게 짰는지 적어 둔 주석이 곧 그 함수의 문서입니다.
# =====================================================================

import csv
import math
import os
import re
import tempfile

import yaml

# label_waypoints.py에서 옮겨 옴. 이 블록은 스튜디오가 통째로 재작성합니다.
LABELS_HEADER = (
    "labels:\n"
    "  # 이 블록은 waypoint studio가 통째로 재작성합니다."
    " 손으로 쓴 주석을 여기 두지 마세요.\n")

# 레코더가 쓰는 컬럼. 손으로 옮긴 점에서는 gps/imu/공분산이 전부 거짓말이 되므로
# 웨이포인트 파일에는 이것만 남깁니다(waypoint_recorder_node.cpp와 같은 목록).
COURSE_FIELDS = ("idx", "x", "y", "yaw", "frame_id")


# --------------------------------------------------------------- 웨이포인트 CSV

class Course:
    """CSV 한 개에서 읽어 온 코스. 좌표는 map 프레임 미터입니다."""

    def __init__(self, path, xs, ys, yaws, has_yaw, frame_id):
        self.path = path
        self.xs = xs                # [float]
        self.ys = ys                # [float]
        self.yaws = yaws            # [float] -- has_yaw가 False인 자리는 유도값
        self.has_yaw = has_yaw      # [bool]  -- CSV에 yaw가 실제로 있었는지
        self.frame_id = frame_id

    def __len__(self):
        return len(self.xs)


def read_course(path):
    """웨이포인트 CSV -> Course.

    헤더 이름으로 읽습니다(위치가 아니라). 그래야 23컬럼짜리 옛 녹화본과 5컬럼짜리
    새 녹화본이 같은 코드로 열립니다 -- path_loader.hpp의 load_waypoint_csv와 같은
    규칙이라, 스튜디오에서 열리는 파일은 미션 매니저에서도 열립니다.
    """
    xs, ys, yaws, has_yaw = [], [], [], []
    frame_id = "map"
    with open(path, newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        names = reader.fieldnames or []
        if "x" not in names or "y" not in names:
            raise ValueError(f"'{os.path.basename(path)}'에 x,y 열이 없습니다.")
        for row in reader:
            try:
                x = float(row["x"])
                y = float(row["y"])
            except (KeyError, TypeError, ValueError):
                # 좌표가 빈 줄(EKF 없이 기록된 줄)은 건너뜁니다.
                continue
            raw_yaw = (row.get("yaw") or "").strip()
            try:
                yaw = float(raw_yaw)
                ok = math.isfinite(yaw)
            except ValueError:
                yaw, ok = 0.0, False
            xs.append(x)
            ys.append(y)
            yaws.append(yaw if ok else 0.0)
            has_yaw.append(ok)
            frame_id = (row.get("frame_id") or frame_id).strip() or frame_id
    if not xs:
        raise ValueError(f"'{os.path.basename(path)}'에 쓸 만한 웨이포인트가 없습니다.")

    # yaw가 없던 자리는 이웃에서 유도해 채워 둡니다. 화면에 방향을 그리려면 값이
    # 있어야 하고, 저장할 때는 has_yaw를 보고 원래 없던 자리를 구분합니다.
    for i, ok in enumerate(has_yaw):
        if not ok:
            yaws[i] = _direction_at(xs, ys, i)
    return Course(path, xs, ys, yaws, has_yaw, frame_id)


def _direction_at(xs, ys, i):
    """i번 점의 진행 방향. 양끝은 한쪽 차분으로 떨어집니다."""
    n = len(xs)
    if n < 2:
        return 0.0
    if 0 < i < n - 1:
        return math.atan2(ys[i + 1] - ys[i - 1], xs[i + 1] - xs[i - 1])
    if i == 0:
        return math.atan2(ys[1] - ys[0], xs[1] - xs[0])
    return math.atan2(ys[-1] - ys[-2], xs[-1] - xs[-2])


def write_course(path, xs, ys, yaws, frame_id="map"):
    """코스를 idx,x,y,yaw,frame_id로 씁니다. 같은 디렉터리 임시 파일 + os.replace.

    임시 파일을 거치는 이유: 저장 도중에 죽어도 반쯤 쓰인 코스가 남지 않게 하려는
    것입니다. 코스 한 개를 다시 따려면 차를 몰고 트랙에 나가야 합니다.

    맨 위에 아무것도 덧붙이지 않습니다. 이 CSV를 읽는 쪽은 전부 첫 줄을 헤더로
    삼으므로(path_loader.hpp의 load_waypoint_csv, 여기 read_course의 csv.DictReader),
    '#'로 시작하는 줄이 하나라도 앞에 있으면 그게 헤더가 되어 x/y 컬럼을 못 찾습니다
    -- 저장한 코스가 미션에서도, 이 프로그램에서도 안 열립니다.
    """
    if not (len(xs) == len(ys) == len(yaws)):
        raise ValueError("xs, ys, yaws의 길이가 다릅니다.")

    directory = os.path.dirname(os.path.abspath(path)) or "."
    os.makedirs(directory, exist_ok=True)
    handle = tempfile.NamedTemporaryFile(
        "w", delete=False, dir=directory, prefix=".studio-", suffix=".csv",
        newline="", encoding="utf-8")
    try:
        with handle:
            writer = csv.writer(handle)
            writer.writerow(COURSE_FIELDS)
            for i, (x, y, yaw) in enumerate(zip(xs, ys, yaws)):
                writer.writerow([i, f"{x:.8f}", f"{y:.8f}", f"{yaw:.8f}", frame_id])
        os.replace(handle.name, path)
    except BaseException:
        # 임시 파일을 남기지 않습니다 -- 다음 실행에서 코스 목록에 섞여 보입니다.
        try:
            os.unlink(handle.name)
        except OSError:
            pass
        raise
    return path


# ------------------------------------------------------------------ mission.yaml

def load_mission(path):
    """mission.yaml -> (raw_text, doc, required_labels, positions).

    raw_text를 들고 다니는 이유가 이 파일의 핵심입니다. 저장할 때 PyYAML로 다시
    쓰면 mission_sim.yaml의 300줄짜리 튜닝 주석이 전부 날아가므로, labels 블록만
    바이트 단위로 갈아끼웁니다(splice_labels).
    """
    with open(path, encoding="utf-8") as handle:
        text = handle.read()
    doc = yaml.safe_load(text) or {}

    # 찍어야 할 라벨 목록은 하드코딩이 아니라 steps가 until로 참조하는 이름입니다.
    # 스텝을 추가하면 스튜디오가 자동으로 그 위치를 요구합니다.
    required = []
    for step in doc.get("steps") or []:
        if isinstance(step, dict) and step.get("until") not in (None, ""):
            if step["until"] not in required:
                required.append(step["until"])

    positions = {}
    for name, value in (doc.get("labels") or {}).items():
        if isinstance(value, dict) and "x" in value and "y" in value:
            positions[name] = (float(value["x"]), float(value["y"]))
    return text, doc, required, positions


def render_labels_block(positions, indices, order):
    """labels 블록을 직렬화합니다. 어떤 step도 참조하지 않는 orphan은 맨 뒤에 --
    조용히 사라지면 지울 기회가 없으므로 눈에 보이게 남깁니다."""
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
    """text의 최상위 `labels:` 매핑을 block으로 교체합니다. 나머지 바이트는 그대로."""
    lines = text.splitlines(keepends=True)
    start = None
    for i, line in enumerate(lines):
        if re.match(r"^labels:\s*(#.*)?$", line):
            start = i
            break

    if start is None:
        # labels 키가 아직 없으면 steps: 바로 위에 넣고, 그것도 없으면 끝에 붙입니다.
        for i, line in enumerate(lines):
            if re.match(r"^steps:\s*(#.*)?$", line):
                return "".join(lines[:i]) + block + "\n" + "".join(lines[i:])
        return text + ("" if text.endswith("\n") else "\n") + block

    # 블록은 다음 0열 시작 줄(형제 키, 또는 그 키를 소개하는 주석)까지입니다.
    # 뒤따르는 빈 줄은 구분자에 속하지 우리 것이 아닙니다.
    end = start + 1
    while end < len(lines) and (lines[end].strip() == "" or lines[end][:1] in " \t"):
        end += 1
    while end > start + 1 and lines[end - 1].strip() == "":
        end -= 1
    return "".join(lines[:start]) + block + "".join(lines[end:])


def save_mission(path, original_text, block):
    """labels 블록만 갈아끼워 저장합니다. 원자적으로 씁니다.

    바이트 스플라이스는 그 바이트를 읽은 시점의 파일에 대해서만 유효하므로,
    연 뒤에 파일이 밖에서 바뀌었으면 거부합니다 -- 그대로 쓰면 남의 편집을
    통째로 되돌려 놓습니다. 호출자가 다시 읽을지 덮어쓸지 물어봅니다.
    """
    with open(path, encoding="utf-8") as handle:
        on_disk = handle.read()
    if on_disk != original_text:
        raise FileChangedError(path)

    updated = splice_labels(original_text, block)
    directory = os.path.dirname(os.path.abspath(path)) or "."
    handle = tempfile.NamedTemporaryFile(
        "w", delete=False, dir=directory, prefix=".studio-", suffix=".yaml",
        encoding="utf-8")
    try:
        with handle:
            handle.write(updated)
        os.replace(handle.name, path)
    except BaseException:
        try:
            os.unlink(handle.name)
        except OSError:
            pass
        raise
    return updated


class FileChangedError(Exception):
    """연 뒤에 파일이 디스크에서 바뀌었습니다."""

    def __init__(self, path):
        super().__init__(f"'{os.path.basename(path)}'가 연 뒤에 바뀌었습니다.")
        self.path = path


# ------------------------------------------------------------------ 배경 이미지

def alignment_path(image_path):
    """이미지의 map 프레임 배치가 든 사이드카. 예: real_course.align.yaml"""
    return os.path.splitext(image_path)[0] + ".align.yaml"


def load_alignment(image_path, xs, ys):
    """image_path의 배치를 (cx, cy, width_m, rot_deg)로 돌려줍니다.

    항공사진에는 지오레퍼런스가 전혀 없으므로 배치는 사이드카에 있습니다. 사이드카가
    아직 없으면 적어도 화면에 웨이포인트와 같이 보이도록 추정값을 씁니다.

    축척을 픽셀당 미터가 아니라 이미지 가로 폭(m)으로 저장하는 이유: 배경은
    load_background에서 다운샘플되므로, 픽셀 기준 축척이면 그 배율만 바꿔도 배경이
    조용히 어긋납니다.
    """
    path = alignment_path(image_path)
    if os.path.exists(path):
        with open(path, encoding="utf-8") as handle:
            doc = yaml.safe_load(handle) or {}
        return (float(doc["center_x"]), float(doc["center_y"]),
                float(doc["width_m"]), float(doc.get("rotation_deg", 0.0)))

    if not xs:
        return (0.0, 0.0, 100.0, 0.0)
    return (float((min(xs) + max(xs)) / 2.0), float((min(ys) + max(ys)) / 2.0),
            1.4 * max(float(max(xs) - min(xs)), 1.0), 0.0)


def save_alignment(image_path, cx, cy, width_m, rot_deg):
    path = alignment_path(image_path)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("# waypoint studio의 정렬 모드가 저장한 배경 이미지 위치입니다.\n")
        handle.write(f"# image: {os.path.basename(image_path)}\n")
        handle.write("# 이미지 중심의 map 좌표(m), 이미지 가로 폭(m), 반시계 회전각(도).\n")
        handle.write(f"center_x: {cx:.4f}\n")
        handle.write(f"center_y: {cy:.4f}\n")
        handle.write(f"width_m: {width_m:.4f}\n")
        handle.write(f"rotation_deg: {rot_deg:.4f}\n")
    return path


def read_obj_extent(obj_path):
    """텍스처가 붙은 바닥 쿼드의 [x0, x1, y0, y1] 경계.

    ground.obj는 텍스처를 쿼드에 모서리끼리 매핑하므로(vt 0..1) 정점 경계가 곧
    이미지의 map 프레임 발자국입니다. 텍스처의 픽셀 종횡비는 쿼드와 다른데,
    Gazebo가 늘려 붙이므로 같은 경계를 그대로 쓰면 같은 방식으로 늘어납니다.
    """
    xs, ys = [], []
    with open(obj_path, encoding="utf-8") as handle:
        for line in handle:
            if line.startswith("v "):
                parts = line.split()
                xs.append(float(parts[1]))
                ys.append(float(parts[2]))
    if not xs:
        raise ValueError(f"'{obj_path}'에 정점이 없습니다.")
    return [min(xs), max(xs), min(ys), max(ys)]


def load_background(path, max_px=2500):
    """배경 이미지를 (RGB bytes, width, height)로. 팬/줌이 느려지지 않게 줄입니다.

    Gazebo 코스 텍스처는 3937 x 4492입니다. 그대로 QPixmap으로 올리면 매 repaint가
    기어가고, 이 툴에 필요한 ~10 cm 배치 정밀도에는 다운샘플로도 한참 넘칩니다.
    """
    from PIL import Image

    image = Image.open(path).convert("RGB")
    scale = min(1.0, max_px / max(image.size))
    if scale < 1.0:
        image = image.resize(
            (max(1, round(image.width * scale)), max(1, round(image.height * scale))),
            Image.BILINEAR)
    return image.tobytes(), image.width, image.height
