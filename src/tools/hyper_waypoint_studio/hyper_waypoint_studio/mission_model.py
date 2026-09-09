#!/usr/bin/env python3
# =====================================================================
# 열려 있는 mission.yaml 하나. 라벨 편집과 스텝 목록을 담당합니다.
#
# 저장은 labels 블록만 바이트로 갈아끼웁니다(formats.save_mission). 라벨은
# 웨이포인트 idx가 아니라 map 좌표로 저장되므로 코스를 다시 녹화해도 살아남습니다.
#
# 라벨은 **코스마다 독립**입니다(mission_loader.hpp와 같은 규칙). 최상위 labels:는
# main의 것이고, courses.<n>.labels는 그 갈래의 것이며, 같은 이름이 두 코스에 있어도
# 됩니다. 그래서 이 모델 안에서 라벨을 가리키는 것은 이름이 아니라 (코스, 이름)이고,
# 그 쌍을 문자열 하나로 만든 것이 formats.label_key입니다.
# =====================================================================

import os

from . import formats


class MissionModel:

    def __init__(self, path, text, doc, required, positions, sentinels):
        self.path = path
        self.text = text
        self.doc = doc
        # 아래 셋은 전부 코스 이름으로 묶인 dict입니다.
        self.required = required          # course -> [steps가 until로 참조하는 이름]
        self.positions = {c: dict(v) for c, v in positions.items()}   # course -> {name: (x, y)}
        self.sentinels = {c: set(v) for c, v in sentinels.items()}    # course -> {name} (`last`)
        self._saved = self._snapshot()

    @classmethod
    def load(cls, path):
        return cls(path, *formats.load_mission(path))

    def _snapshot(self):
        return {c: dict(v) for c, v in self.positions.items()}

    @property
    def name(self):
        return os.path.basename(self.path)

    @property
    def dirty(self):
        return self.positions != self._saved

    @property
    def snap_tolerance(self):
        try:
            return float(self.doc.get("label_snap_tolerance_m", 1.0))
        except (TypeError, ValueError):
            return 1.0

    @property
    def course_names(self):
        """미션이 아는 코스 이름들. main은 waypoint_csv 하나를 가리킵니다."""
        return ["main"] + sorted((self.doc.get("courses") or {}).keys())

    def labels_for_course(self, course_name):
        """그 코스에 붙는 라벨 이름들. 배치해야 할 것(required)이 먼저이고,
        어떤 step도 참조하지 않는 orphan이 뒤에 옵니다."""
        names = list(self.required.get(course_name, []))
        known = set(self.positions.get(course_name, {})) | set(
            self.sentinels.get(course_name, set()))
        return names + sorted(n for n in known if n not in names)

    def is_sentinel(self, name, course_name):
        """`last` 센티널인지. 좌표가 아니라 '그 코스의 마지막 점'이라는 뜻이고,
        로더도 거리 검사를 건너뛰므로 여기서도 건너뜁니다. 옮길 수도 없습니다."""
        return name in self.sentinels.get(course_name, set())

    # ------------------------------------------------------------------ 스텝
    def steps(self):
        """(index, type, until, course, route) 목록.

        mission_loader는 steps를 먼저 펼치고(인덱스 0..N-1이 yaml 순서와 같습니다)
        routes의 스텝을 그 뒤에 덧붙입니다. 그래서 main 스텝의 인덱스는 여기서
        그대로 셀 수 있지만, route 스텝의 인덱스는 셀 수 없습니다 -- 그쪽은
        mission_manager가 내보내는 ~/steps 목록을 받아 씁니다(drive_panel).
        """
        out = []
        for i, step in enumerate(self.doc.get("steps") or []):
            if not isinstance(step, dict):
                continue
            out.append((i, step.get("type", "?"), step.get("until", ""),
                        step.get("course", "main"), ""))
        return out

    def step_summary(self, step):
        """mission_manager의 status_text()와 같은 모양으로 한 줄."""
        index, kind, until, course, route = step
        text = f"[{index + 1}] {kind}"
        if kind == "drive":
            text += f" until={until}"
            if course and course != "main":
                text += f" course={course}"
        elif kind == "stop":
            duration = (self.doc.get("steps") or [])[index].get("duration_s")
            if duration is not None:
                text += f" {duration}s"
        elif kind == "wait_signal":
            value = (self.doc.get("steps") or [])[index].get("value")
            if value:
                text += f" value={value}"
        elif kind == "branch":
            text += " (branch)"
        if route:
            text += f"  <{route}>"
        return text

    # ------------------------------------------------------------------ 라벨
    def position(self, course_name, name):
        return self.positions.get(course_name, {}).get(name)

    def place(self, course_name, name, x, y):
        self.positions.setdefault(course_name, {})[name] = (float(x), float(y))
        # 좌표를 찍었으면 더 이상 `last`가 아닙니다. 둘 다 남으면 저장할 때 어느 쪽을
        # 써야 할지 모호해집니다.
        self.sentinels.get(course_name, set()).discard(name)

    def remove(self, course_name, name):
        self.positions.get(course_name, {}).pop(name, None)

    def snap_report(self, courses):
        """라벨별 (course, name, index, distance, state)를 돌려줍니다.

        courses는 {미션 코스 이름: formats.Course 또는 None}입니다 -- 라벨은 자기 코스의
        CSV에만 스냅되므로(mission_loader.snap_labels) 코스마다 따로 재야 합니다.

        state: 'ok' | 'near' | 'over' | 'missing' | 'nocourse' | 'sentinel'
        mission_loader.snap_labels는 거리가 tolerance를 넘으면 미션 전체를 거부하므로,
        저장하기 전에 그 선을 넘었는지 눈에 보여야 합니다.
        """
        tol = self.snap_tolerance
        report = []
        for course_name in self.course_names:
            course = courses.get(course_name)
            for name in self.labels_for_course(course_name):
                if self.is_sentinel(name, course_name):
                    report.append((course_name, name, None, None, "sentinel"))
                    continue
                point = self.position(course_name, name)
                if point is None:
                    report.append((course_name, name, None, None, "missing"))
                    continue
                if course is None or len(course) == 0:
                    report.append((course_name, name, None, None, "nocourse"))
                    continue
                index, distance = course.nearest(*point)
                if distance >= tol:
                    state = "over"
                elif distance >= 0.5 * tol:
                    state = "near"
                else:
                    state = "ok"
                report.append((course_name, name, index, distance, state))
        return report

    def orphans(self):
        """(course, name) 목록. 어떤 step도 until로 참조하지 않는 라벨입니다."""
        out = []
        for course_name in self.course_names:
            required = set(self.required.get(course_name, []))
            known = set(self.positions.get(course_name, {})) | set(
                self.sentinels.get(course_name, set()))
            out.extend((course_name, n) for n in sorted(known - required))
        return out

    # ------------------------------------------------------------------ 저장
    def save(self, courses):
        """courses는 snap_report와 같은 {코스 이름: formats.Course 또는 None}입니다.

        라벨이 하나라도 있는 코스마다 블록을 하나씩 만들어 한 번에 갈아끼웁니다.
        코스를 안 연 자리는 wp 주석만 빠지고 좌표는 그대로 다시 쓰입니다 -- 열지 않은
        갈래의 라벨을 저장이 지우면 안 됩니다.
        """
        blocks = []
        for course_name in self.course_names:
            positions = self.positions.get(course_name, {})
            sentinels = self.sentinels.get(course_name, set())
            if not positions and not sentinels:
                continue
            course = courses.get(course_name)
            indices = {}
            if course is not None and len(course):
                for name, (x, y) in positions.items():
                    indices[name] = course.nearest(x, y)[0]
            blocks.append((course_name, formats.render_labels_block(
                positions, indices, self.required.get(course_name, []), sentinels)))
        self.text = formats.save_mission(self.path, self.text, blocks)
        self._saved = self._snapshot()
        return self.path

    def reload(self):
        (self.text, self.doc, self.required,
         positions, sentinels) = formats.load_mission(self.path)
        self.positions = {c: dict(v) for c, v in positions.items()}
        self.sentinels = {c: set(v) for c, v in sentinels.items()}
        self._saved = self._snapshot()
