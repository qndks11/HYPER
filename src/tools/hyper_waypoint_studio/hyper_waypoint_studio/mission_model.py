#!/usr/bin/env python3
# =====================================================================
# 열려 있는 mission.yaml 하나. 라벨 편집과 스텝 목록을 담당합니다.
#
# 저장은 labels 블록만 바이트로 갈아끼웁니다(formats.save_mission). 라벨은
# 웨이포인트 idx가 아니라 map 좌표로 저장되므로 코스를 다시 녹화해도 살아남습니다.
# =====================================================================

import os

from . import formats


class MissionModel:

    def __init__(self, path, text, doc, required, positions):
        self.path = path
        self.text = text
        self.doc = doc
        self.required = required          # steps가 until로 참조하는 이름, 등장 순서
        self.positions = dict(positions)  # name -> (x, y)
        self._saved_positions = dict(positions)

    @classmethod
    def load(cls, path):
        text, doc, required, positions = formats.load_mission(path)
        return cls(path, text, doc, required, positions)

    @property
    def name(self):
        return os.path.basename(self.path)

    @property
    def dirty(self):
        return self.positions != self._saved_positions

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
        """그 코스에 붙는 라벨 이름들.

        최상위 labels는 main의 것이고, courses.<n>.labels는 그 갈래의 것입니다.
        스냅 거리를 엉뚱한 코스에 대고 재지 않으려면 이 구분이 필요합니다.
        """
        if course_name == "main":
            return list(self.positions.keys())
        course = (self.doc.get("courses") or {}).get(course_name) or {}
        return list((course.get("labels") or {}).keys())

    def is_sentinel(self, name, course_name):
        """`last` 센티널인지. 좌표가 아니라 '그 코스의 마지막 점'이라는 뜻이고,
        로더도 거리 검사를 건너뛰므로 여기서도 건너뜁니다."""
        if course_name == "main":
            return self.doc.get("labels", {}).get(name) == "last"
        course = (self.doc.get("courses") or {}).get(course_name) or {}
        return (course.get("labels") or {}).get(name) == "last"

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
    def place(self, name, x, y):
        self.positions[name] = (float(x), float(y))

    def remove(self, name):
        self.positions.pop(name, None)

    def snap_report(self, course):
        """라벨별 (name, index, distance, state)를 돌려줍니다.

        state: 'ok' | 'near' | 'over' | 'missing'
        mission_loader.snap_labels는 거리가 tolerance를 넘으면 미션 전체를 거부하므로,
        저장하기 전에 그 선을 넘었는지 눈에 보여야 합니다.
        """
        tol = self.snap_tolerance
        report = []
        for name in self.required:
            if name not in self.positions:
                report.append((name, None, None, "missing"))
                continue
            x, y = self.positions[name]
            if course is None or len(course) == 0:
                report.append((name, None, None, "nocourse"))
                continue
            index, distance = course.nearest(x, y)
            if distance >= tol:
                state = "over"
            elif distance >= 0.5 * tol:
                state = "near"
            else:
                state = "ok"
            report.append((name, index, distance, state))
        return report

    def orphans(self):
        return [n for n in self.positions if n not in self.required]

    # ------------------------------------------------------------------ 저장
    def save(self, course):
        indices = {}
        if course is not None and len(course):
            for name, (x, y) in self.positions.items():
                indices[name] = course.nearest(x, y)[0]
        block = formats.render_labels_block(self.positions, indices, self.required)
        self.text = formats.save_mission(self.path, self.text, block)
        self._saved_positions = dict(self.positions)
        return self.path

    def reload(self):
        text, doc, required, positions = formats.load_mission(self.path)
        self.text, self.doc = text, doc
        self.required, self.positions = required, dict(positions)
        self._saved_positions = dict(positions)
