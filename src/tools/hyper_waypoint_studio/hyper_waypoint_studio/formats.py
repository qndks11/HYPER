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


# 콘은 라벨이 아닙니다(steps[].cases[].cone -- courses.<n>.labels가 아니라). case의
# value 문자열이 미션 전체에서 유일하다는 보장이 없으므로, 이름이 아니라 위치
# (scope, step_index, case_index)로 키를 만듭니다.
def cone_key(scope, step_index, case_index):
    tag = scope if scope == "steps" else f"routes{KEY_SEPARATOR}{scope[1]}"
    return f"{tag}{KEY_SEPARATOR}{step_index}{KEY_SEPARATOR}{case_index}"


def split_cone_key(key):
    parts = key.split(KEY_SEPARATOR)
    if parts[0] == "steps":
        return "steps", int(parts[1]), int(parts[2])
    return ("routes", parts[1]), int(parts[2]), int(parts[3])


def resolve_asset(given, bases):
    """미션이 적은 경로(`track/common_1.csv`, `real_course.png`)를 실제 파일로 풉니다.

    mission_loader.hpp의 resolve_csv_path와 같은 규칙입니다 -- 절대 경로면 그대로 쓰고,
    상대 경로면 bases를 앞에서부터 붙여 보고 **처음 실재하는 것**을 돌려줍니다. 미션에
    파일 이름만 적을 수 있어야 하는 이유가 이것입니다: 코스 CSV는 waypoints/ 아래에,
    배경 이미지는 hyper_gazebo의 meshes/ 아래에 있어 미션 파일과 폴더가 다릅니다.

    아무 데서도 못 찾으면 None입니다 -- 부르는 쪽이 어느 파일이 없는지 한 번에 모아
    보여 줍니다(없는 파일마다 모달을 띄우면 코스 열한 개짜리 미션에서 열한 번 뜹니다).
    """
    if not given:
        return None
    path = os.path.expanduser(str(given))
    if os.path.isabs(path):
        return path if os.path.exists(path) else None
    for base in bases:
        if not base:
            continue
        candidate = os.path.join(base, path)
        if os.path.exists(candidate):
            return os.path.abspath(candidate)
    return None


def load_mission(path):
    """mission.yaml -> (raw_text, doc, required, positions, sentinels, cones).

    raw_text를 들고 다니는 이유가 이 파일의 핵심입니다. 저장할 때 PyYAML로 다시
    쓰면 mission_track.yaml의 수백 줄짜리 튜닝 주석이 전부 날아가므로, labels 블록만
    바이트 단위로 갈아끼웁니다(splice_labels). 콘도 같은 이유로 값 하나만
    갈아끼웁니다(splice_cone).

    required / positions / sentinels는 전부 **코스 이름으로 묶인** dict입니다.
    최상위 labels:는 main의 것이고, courses.<n>.labels는 그 갈래의 것입니다 --
    mission_loader가 라벨을 그 코스의 CSV에만 스냅하므로, 스냅 거리를 엉뚱한 코스에
    대고 재지 않으려면 여기서부터 갈라 놓아야 합니다.

    sentinels는 좌표가 아니라 `last`로 적힌 라벨입니다("그 코스의 마지막 점").
    좌표가 없으니 화면에 찍을 수도 옮길 수도 없지만, **저장할 때 반드시 도로
    써 줘야 합니다** -- 안 그러면 저장 한 번에 파일에서 사라지고 미션이 로드되지
    않습니다.

    cones는 cone_key(scope, step_index, case_index) -> {x, y, value, radius,
    scope, step_index, case_index}입니다. 라벨과 달리 코스에 묶이지 않고(생좌표라
    어떤 CSV에도 스냅되지 않습니다), 좌표 없이는 존재할 수 없으므로 sentinel도
    없습니다.
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

    # 콘은 라벨과 완전히 다른 자리(steps[].cases[].cone)에 있습니다. required 스캔과
    # 같은 순서(steps 먼저, 그다음 routes)로 훑어야 splice_cone의 step_index가
    # save_mission이 다시 읽을 때와 항상 같은 스텝을 가리킵니다.
    cones = {}

    def note_branch(scope, step_index, step):
        if not isinstance(step, dict) or step.get("type") != "branch":
            return
        for case_index, case in enumerate(step.get("cases") or []):
            if not isinstance(case, dict):
                continue
            cone = case.get("cone")
            if not isinstance(cone, dict) or "x" not in cone or "y" not in cone:
                continue
            key = cone_key(scope, step_index, case_index)
            cones[key] = {
                "x": float(cone["x"]),
                "y": float(cone["y"]),
                "value": case.get("value", ""),
                "radius": cone.get("radius", step.get("cone_radius")),
                "scope": scope,
                "step_index": step_index,
                "case_index": case_index,
            }

    for i, step in enumerate(doc.get("steps") or []):
        note_branch("steps", i, step)
    for route_name, route in (doc.get("routes") or {}).items():
        for i, step in enumerate(route or []):
            note_branch(("routes", route_name), i, step)

    return text, doc, required, positions, sentinels, cones


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


def _first_dash_indent(lines, lo, hi):
    """[lo, hi)에서 첫 `- ` 항목의 들여쓰기 칸 수. 없으면 None.

    부모 키(`steps:`, `cases:`, ...)의 들여쓰기에서 몇 칸을 더 들여썼는지는 파일마다
    또는 코스 갈래마다 다를 수 있어 고정폭으로 가정하지 않고 실제 줄에서 잽니다.
    """
    for i in range(lo, hi):
        match = re.match(r"^(\s*)- ", lines[i])
        if match:
            return len(match.group(1))
    return None


def _list_item_ranges(lines, list_indent, lo, hi):
    """[lo, hi)에서 정확히 list_indent 칸 들여쓴 `- `로 시작하는 각 항목의 [start, end)
    범위 목록. 한 줄짜리 flow 항목(`- {a: 1}`)과 여러 줄짜리 block 항목(`- type: x\\n
    ...`) 둘 다에 씁니다 -- block 항목의 이어지는 줄은 항상 그 항목의 대시보다 더
    깊이 들여쓰이므로, 다음 `- `가 나올 때까지가 그 항목입니다.
    """
    pattern = re.compile(r"^" + " " * list_indent + r"- ")
    starts = [i for i in range(lo, hi) if pattern.match(lines[i])]
    ranges = []
    for idx, start in enumerate(starts):
        end = starts[idx + 1] if idx + 1 < len(starts) else hi
        while end > start + 1 and lines[end - 1].strip() == "":
            end -= 1
        ranges.append((start, end))
    return ranges


def splice_cone(text, scope, step_index, case_index, x, y, expected_value=None):
    """text에서 한 콘의 x/y만 자리에서 바꿔치기합니다. 나머지 바이트는 그대로.

    라벨과 달리 콘은 courses.<n>.labels 같은 자기 블록이 없습니다 -- steps[].cases[]
    또는 routes.<name>[].cases[] 안, 한 줄짜리 flow 매핑(`- {value: ..., cone: {x:
    .., y: ..}}`) 속에 파묻혀 있으므로, 블록 전체가 아니라 그 줄의 cone: {...}
    부분문자열만 고쳐야 value/goto/주석이 그대로 남습니다.

    scope는 "steps" 또는 ("routes", 이름)입니다. expected_value를 주면 찾아낸 case가
    그 value를 갖고 있는지 확인하고, 아니면 인덱스가 어긋난 것이므로 조용히 엉뚱한
    자리를 고치는 대신 ValueError를 냅니다.

    들여쓰기는 어디서도 고정폭으로 가정하지 않고 실제 줄에서 잽니다 -- steps:의 항목
    들여쓰기와 routes.<name>:의 항목 들여쓰기가 갈래마다 다를 수 있기 때문입니다.
    """
    lines = text.splitlines(keepends=True)

    if scope == "steps":
        owner = _key_line(lines, "steps", 0, 0, len(lines))
        if owner is None:
            raise ValueError("mission.yaml에 최상위 'steps:' 블록이 없습니다.")
        list_lo, list_hi = owner + 1, _body_end(lines, owner, 0, len(lines))
    else:
        _, route_name = scope
        routes = _key_line(lines, "routes", 0, 0, len(lines))
        if routes is None:
            raise ValueError("mission.yaml에 최상위 'routes:' 블록이 없습니다.")
        routes_end = _body_end(lines, routes, 0, len(lines))
        entry = entry_indent = None
        pattern = re.compile(r"^(\s+)" + re.escape(route_name) + r":\s*(#.*)?$")
        for i in range(routes + 1, routes_end):
            match = pattern.match(lines[i])
            if match:
                entry, entry_indent = i, len(match.group(1))
                break
        if entry is None:
            raise ValueError(f"'routes:' 아래에 '{route_name}:' 항목이 없습니다.")
        list_lo, list_hi = entry + 1, _body_end(lines, entry, entry_indent, routes_end)

    list_indent = _first_dash_indent(lines, list_lo, list_hi)
    if list_indent is None:
        raise ValueError("스텝 목록에서 '- ' 항목을 찾지 못했습니다.")
    step_ranges = _list_item_ranges(lines, list_indent, list_lo, list_hi)
    if not (0 <= step_index < len(step_ranges)):
        raise ValueError(f"스텝 인덱스 {step_index}가 범위를 벗어났습니다.")
    step_start, step_end = step_ranges[step_index]

    cases = None
    cases_pattern = re.compile(r"^(\s+)cases:\s*(#.*)?$")
    for i in range(step_start, step_end):
        match = cases_pattern.match(lines[i])
        if match:
            cases, cases_indent = i, len(match.group(1))
            break
    if cases is None:
        raise ValueError("그 스텝에 'cases:'가 없습니다.")
    cases_body_lo, cases_body_hi = cases + 1, _body_end(lines, cases, cases_indent, step_end)

    item_indent = _first_dash_indent(lines, cases_body_lo, cases_body_hi)
    if item_indent is None:
        raise ValueError("그 스텝의 'cases:'에 항목이 없습니다.")

    case_ranges = _list_item_ranges(lines, item_indent, cases_body_lo, cases_body_hi)
    if not (0 <= case_index < len(case_ranges)):
        raise ValueError(f"케이스 인덱스 {case_index}가 범위를 벗어났습니다.")
    case_start, case_end = case_ranges[case_index]
    case_text = "".join(lines[case_start:case_end])

    if expected_value is not None:
        value_pattern = re.compile(r"value:\s*[\"']?" + re.escape(str(expected_value)))
        if not value_pattern.search(case_text):
            raise ValueError(
                f"'{expected_value}' 케이스를 찾지 못했습니다 -- 인덱스가 어긋났습니다.")

    cone_pattern = re.compile(r"cone:\s*\{[^{}]*\}")
    cone_match = cone_pattern.search(case_text)
    if cone_match is None:
        raise ValueError("그 케이스에서 'cone: {...}' 한 줄짜리 표기를 찾지 못했습니다.")
    inner = cone_match.group(0)
    inner = re.sub(r"(\bx:\s*)-?\d+(?:\.\d+)?", lambda m: m.group(1) + f"{x:.2f}",
                    inner, count=1)
    inner = re.sub(r"(\by:\s*)-?\d+(?:\.\d+)?", lambda m: m.group(1) + f"{y:.2f}",
                    inner, count=1)
    new_case_text = case_text[:cone_match.start()] + inner + case_text[cone_match.end():]

    return "".join(lines[:case_start]) + new_case_text + "".join(lines[case_end:])


# ------------------------------------------------------------------ 진입 금지 구역

# 최상위 keepout: 블록의 머리. labels와 같은 이유로 블록 **안에** 둡니다 -- 키 줄 위에 두면
# 블록 밖이라 저장할 때마다 한 줄씩 늘어납니다.
KEEPOUT_HEADER = (
    "keepout:\n"
    "  # 진입 금지 구역(map 프레임 다각형, 꼭짓점 3개 이상). 이 블록은 waypoint studio가\n"
    "  # 통째로 재작성합니다. 손으로 쓴 주석을 여기 두지 마세요.\n")

KEEPOUT_EMPTY = "keepout: []\n"


def load_keepout(doc):
    """doc의 최상위 keepout: -> [{"name": str, "points": [(x, y), ...]}].

    mission_loader.hpp의 load_keepout과 같은 모양만 받습니다. 모양이 틀린 항목을 조용히
    버리지 않고 ValueError를 냅니다 -- 버린 채로 저장하면 그 구역이 파일에서 사라집니다.
    꼭짓점이 3개 미만인 구역은 받습니다(로더는 거부하지만, 스튜디오에서 고칠 수는 있어야
    합니다).
    """
    raw = doc.get("keepout")
    if raw in (None, ""):
        return []
    if not isinstance(raw, list):
        raise ValueError("'keepout:'는 {name, points} 목록이어야 합니다.")
    zones = []
    for i, entry in enumerate(raw):
        if not isinstance(entry, dict) or not isinstance(entry.get("points"), list):
            raise ValueError(f"keepout #{i}: 'points: [[x, y], ...]'가 없습니다.")
        points = []
        for point in entry["points"]:
            if not isinstance(point, (list, tuple)) or len(point) != 2:
                raise ValueError(f"keepout #{i}: 꼭짓점은 [x, y]여야 합니다 (받은 값: {point!r}).")
            points.append((float(point[0]), float(point[1])))
        name = entry.get("name")
        zones.append({"name": str(name) if name not in (None, "") else f"zone_{i}",
                      "points": points})
    return zones


def _yaml_name(name):
    """구역 이름을 yaml 스칼라로. 평범한 이름은 그대로, 나머지는 따옴표로 감쌉니다.

    json 문자열은 그대로 yaml의 큰따옴표 스칼라입니다. true/null/숫자처럼 읽히는 이름도
    감싸야 문자열로 남습니다.
    """
    import json

    if re.fullmatch(r"[^\W\d][\w\-]*", name) and name.lower() not in (
            "true", "false", "yes", "no", "on", "off", "null"):
        return name
    return json.dumps(name, ensure_ascii=False)


def render_keepout_block(zones):
    """최상위 keepout: 블록 전체를 직렬화합니다. 구역이 없으면 `keepout: []`.

    꼭짓점은 한 줄짜리 flow 목록입니다 -- 구역 하나가 git diff에서 한 줄로 보이고, 좌표는
    라벨과 같은 cm 단위(:.2f)입니다.
    """
    if not zones:
        return KEEPOUT_EMPTY
    lines = [KEEPOUT_HEADER]
    for zone in zones:
        points = ", ".join(f"[{x:.2f}, {y:.2f}]" for x, y in zone["points"])
        lines.append(f"  - name: {_yaml_name(zone['name'])}\n")
        lines.append(f"    points: [{points}]\n")
    return "".join(lines)


def _keepout_body_end(lines, start):
    """최상위 keepout: 블록의 끝(exclusive).

    _body_end와 거의 같지만, 들여쓰기 없이 쓴 목록(`keepout:` 다음 줄이 바로 `- name:`)도
    yaml로는 맞으므로 칸 0의 `-` 줄까지 블록으로 셉니다. 그걸 빼먹으면 손으로 쓴 파일을
    저장하는 순간 옛 항목이 블록 밖에 남아 yaml이 깨집니다.
    """
    end = start + 1
    while end < len(lines):
        line = lines[end]
        if line.strip() == "" or line[:1] in (" ", "\t") or line.startswith("-"):
            end += 1
            continue
        break
    while end > start + 1 and lines[end - 1].strip() == "":
        end -= 1
    return end


def splice_keepout(text, block):
    """text의 최상위 keepout: 블록을 block으로 교체합니다. 나머지 바이트는 그대로.

    블록이 없으면 steps: 위에 새로 넣습니다 -- steps: 바로 위에 붙은 주석 덩어리보다도
    위에 넣어, 그 주석이 steps에서 떨어지지 않게 합니다. 없는 블록을 비우라는 요청이면
    파일을 건드리지 않습니다.
    """
    lines = text.splitlines(keepends=True)
    pattern = re.compile(r"^keepout:(\s.*)?$")
    start = next((i for i, line in enumerate(lines) if pattern.match(line)), None)
    if start is None:
        if block == KEEPOUT_EMPTY:
            return text
        for i, line in enumerate(lines):
            if re.match(r"^steps:\s*(#.*)?$", line):
                while i > 0 and lines[i - 1].startswith("#"):
                    i -= 1
                return "".join(lines[:i]) + block + "\n" + "".join(lines[i:])
        return text + ("" if text.endswith("\n") else "\n") + block
    end = _keepout_body_end(lines, start)
    return "".join(lines[:start]) + block + "".join(lines[end:])


def save_mission(path, original_text, blocks, cone_edits=(), keepout_block=None):
    """labels 블록들과 콘 좌표들을 갈아끼워 저장합니다. 원자적으로 씁니다.

    blocks는 (코스 이름, 블록) 목록입니다 -- 코스마다 자기 labels 블록이 따로
    있으므로 한 번의 저장이 여러 자리를 건드립니다. cone_edits는 (scope,
    step_index, case_index, x, y, expected_value) 목록입니다. keepout_block이 None이
    아니면 최상위 keepout: 블록도 같은 한 번의 쓰기로 갈아끼웁니다.

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
    for scope, step_index, case_index, x, y, expected_value in cone_edits:
        updated = splice_cone(updated, scope, step_index, case_index, x, y, expected_value)
    if keepout_block is not None:
        updated = splice_keepout(updated, keepout_block)

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
