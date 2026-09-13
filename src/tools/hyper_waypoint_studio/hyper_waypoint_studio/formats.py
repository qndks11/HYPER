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

# 라벨 하나를 가리키는 키. 라벨 이름은 코스마다 독립이라(mission_loader.hpp의 주석)
# 이름만으로는 유일하지 않습니다 -- 코스를 같이 들고 다녀야 합니다. Qt의 UserRole처럼
# 스칼라 하나만 담을 수 있는 자리를 위해 문자열로 만듭니다.
KEY_SEPARATOR = "\x1f"


def label_key(course, name):
    return f"{course}{KEY_SEPARATOR}{name}"


def split_key(key):
    course, _, name = key.partition(KEY_SEPARATOR)
    return course, name


def load_mission(path):
    """mission.yaml -> (raw_text, doc, required, positions, sentinels).

    raw_text를 들고 다니는 이유가 이 파일의 핵심입니다. 저장할 때 PyYAML로 다시
    쓰면 mission_track.yaml의 수백 줄짜리 튜닝 주석이 전부 날아가므로, labels 블록만
    바이트 단위로 갈아끼웁니다(splice_labels).

    required / positions / sentinels는 전부 **코스 이름으로 묶인** dict입니다.
    최상위 labels:는 main의 것이고, courses.<n>.labels는 그 갈래의 것입니다 --
    mission_loader가 라벨을 그 코스의 CSV에만 스냅하므로, 스냅 거리를 엉뚱한 코스에
    대고 재지 않으려면 여기서부터 갈라 놓아야 합니다.

    sentinels는 좌표가 아니라 `last`로 적힌 라벨입니다("그 코스의 마지막 점").
    좌표가 없으니 화면에 찍을 수도 옮길 수도 없지만, **저장할 때 반드시 도로
    써 줘야 합니다** -- 안 그러면 저장 한 번에 파일에서 사라지고 미션이 로드되지
    않습니다.
    """
    with open(path, encoding="utf-8") as handle:
        text = handle.read()
    doc = yaml.safe_load(text) or {}

    # 찍어야 할 라벨 목록은 하드코딩이 아니라 steps가 until로 참조하는 이름입니다.
    # 스텝을 추가하면 스튜디오가 자동으로 그 위치를 요구합니다. routes의 스텝도 같이
    # 봅니다 -- 갈래 안의 drive도 라벨을 참조하고, 그쪽이 갈래 코스의 라벨입니다.
    required = {}

    def note_step(step):
        if not isinstance(step, dict):
            return
        name = step.get("until")
        if name in (None, ""):
            return
        course = step.get("course") or "main"
        names = required.setdefault(course, [])
        if name not in names:
            names.append(name)

    for step in doc.get("steps") or []:
        note_step(step)
    for route in (doc.get("routes") or {}).values():
        for step in route or []:
            note_step(step)

    positions = {}
    sentinels = {}

    def note_labels(course, block):
        for name, value in (block or {}).items():
            if isinstance(value, dict) and "x" in value and "y" in value:
                positions.setdefault(course, {})[name] = (
                    float(value["x"]), float(value["y"]))
            elif value == "last":
                sentinels.setdefault(course, set()).add(name)

    note_labels("main", doc.get("labels"))
    for course, entry in (doc.get("courses") or {}).items():
        if isinstance(entry, dict):
            note_labels(course, entry.get("labels"))

    return text, doc, required, positions, sentinels


def render_labels_block(positions, indices, order, sentinels=()):
    """한 코스의 labels 블록을 직렬화합니다. 항상 들여쓰기 0으로 씁니다 --
    갈래 코스의 블록은 splice_labels가 제자리 들여쓰기를 붙여 줍니다.

    어떤 step도 참조하지 않는 orphan은 맨 뒤에 -- 조용히 사라지면 지울 기회가
    없으므로 눈에 보이게 남깁니다.

    sentinels(`last`)는 좌표가 없지만 그대로 다시 써야 합니다. 빠뜨리면 저장
    한 번에 미션이 로드되지 않습니다.
    """
    lines = [LABELS_HEADER]
    known = set(positions) | set(sentinels)
    orphans = [n for n in known if n not in order]
    for name in list(order) + sorted(orphans):
        if name not in known:
            continue
        # 주석은 하나로 모읍니다. 예전에는 orphan 표시를 값 뒤에 그냥 붙였는데, 그 앞의
        # "# wp #N"이 없으면(= 코스를 안 연 채 저장하면) 주석이 아니라 값의 일부가 되어
        # 저장한 yaml이 파싱조차 안 됐습니다.
        notes = []
        if name in indices:
            notes.append(f"wp #{indices[name]}")
        if name in orphans:
            notes.append("orphan: 어떤 step도 참조하지 않음")
        comment = ("   # " + ", ".join(notes)) if notes else ""
        if name in sentinels:
            lines.append(f"  {name}: last{comment}\n")
            continue
        x, y = positions[name]
        lines.append(f"  {name}: {{x: {x:.3f}, y: {y:.3f}}}{comment}\n")
    return "".join(lines)


def _key_line(lines, key, indent, lo, hi):
    """[lo, hi)에서 정확히 indent 칸 들여쓴 `key:` 줄의 인덱스. 없으면 None."""
    pattern = re.compile(r"^" + " " * indent + re.escape(key) + r":\s*(#.*)?$")
    for i in range(lo, hi):
        if pattern.match(lines[i]):
            return i
    return None


def _body_end(lines, start, indent, hi):
    """start(키 줄)가 소유하는 블록의 끝(exclusive).

    블록은 다음으로 indent 이하로 나오는 줄까지입니다 -- 형제 키든, 그 키를
    소개하는 주석이든. 뒤따르는 빈 줄은 구분자에 속하지 우리 것이 아닙니다.
    """
    end = start + 1
    while end < hi:
        line = lines[end]
        if line.strip() == "":
            end += 1
            continue
        if len(line) - len(line.lstrip(" \t")) <= indent:
            break
        end += 1
    while end > start + 1 and lines[end - 1].strip() == "":
        end -= 1
    return end


def _indented(block, indent):
    if indent <= 0:
        return block
    pad = " " * indent
    return "".join(
        (pad + line if line.strip() else line) for line in block.splitlines(keepends=True))


def splice_labels(text, block, course=None):
    """text에서 한 코스의 labels 매핑을 block으로 교체합니다. 나머지 바이트는 그대로.

    course가 None이거나 'main'이면 최상위 `labels:`, 아니면
    `courses:` -> `<course>:` -> `labels:`입니다. block은 들여쓰기 0으로 받아서
    제자리 들여쓰기를 여기서 붙입니다 -- 호출자가 갈래 블록의 깊이를 알 필요가
    없게 하려는 것입니다.
    """
    lines = text.splitlines(keepends=True)

    if course in (None, "", "main"):
        start = _key_line(lines, "labels", 0, 0, len(lines))
        if start is None:
            # labels 키가 아직 없으면 steps: 바로 위에 넣고, 그것도 없으면 끝에 붙입니다.
            for i, line in enumerate(lines):
                if re.match(r"^steps:\s*(#.*)?$", line):
                    return "".join(lines[:i]) + block + "\n" + "".join(lines[i:])
            return text + ("" if text.endswith("\n") else "\n") + block
        end = _body_end(lines, start, 0, len(lines))
        return "".join(lines[:start]) + block + "".join(lines[end:])

    courses = _key_line(lines, "courses", 0, 0, len(lines))
    if courses is None:
        raise ValueError("mission.yaml에 최상위 'courses:' 블록이 없습니다.")
    courses_end = _body_end(lines, courses, 0, len(lines))

    entry = entry_indent = None
    pattern = re.compile(r"^(\s+)" + re.escape(course) + r":\s*(#.*)?$")
    for i in range(courses + 1, courses_end):
        match = pattern.match(lines[i])
        if match:
            entry, entry_indent = i, len(match.group(1))
            break
    if entry is None:
        raise ValueError(f"'courses:' 아래에 '{course}:' 항목이 없습니다.")
    entry_end = _body_end(lines, entry, entry_indent, courses_end)

    labels = labels_indent = None
    pattern = re.compile(r"^(\s+)labels:\s*(#.*)?$")
    for i in range(entry + 1, entry_end):
        match = pattern.match(lines[i])
        if match and len(match.group(1)) > entry_indent:
            labels, labels_indent = i, len(match.group(1))
            break
    if labels is None:
        # 이 코스에 아직 labels:가 없습니다. 항목 끝에 새로 답니다.
        return ("".join(lines[:entry_end]) + _indented(block, entry_indent + 2)
                + "".join(lines[entry_end:]))
    labels_end = _body_end(lines, labels, labels_indent, entry_end)
    return ("".join(lines[:labels]) + _indented(block, labels_indent)
            + "".join(lines[labels_end:]))


def save_mission(path, original_text, blocks):
    """labels 블록들만 갈아끼워 저장합니다. 원자적으로 씁니다.

    blocks는 (코스 이름, 블록) 목록입니다 -- 코스마다 자기 labels 블록이 따로
    있으므로 한 번의 저장이 여러 자리를 건드립니다.

    바이트 스플라이스는 그 바이트를 읽은 시점의 파일에 대해서만 유효하므로,
    연 뒤에 파일이 밖에서 바뀌었으면 거부합니다 -- 그대로 쓰면 남의 편집을
    통째로 되돌려 놓습니다. 호출자가 다시 읽을지 덮어쓸지 물어봅니다.
    """
    with open(path, encoding="utf-8") as handle:
        on_disk = handle.read()
    if on_disk != original_text:
        raise FileChangedError(path)

    updated = original_text
    for course, block in blocks:
        updated = splice_labels(updated, block, course)

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
