#!/usr/bin/env python3
# =====================================================================
# 화면에 올라온 코스 CSV 하나의 상태. Qt를 import하지 않습니다 -- 편집/실행 취소가
# 위젯 없이도 성립해야 저장 안전성을 따로 시험할 수 있기 때문입니다.
# =====================================================================

import math
import os

from . import formats, geometry


class CourseModel:
    """CSV 하나 = 코스 하나. 좌표는 map 프레임 미터입니다."""

    def __init__(self, path, xs, ys, yaws, frame_id, color):
        self.path = path
        self.xs = list(xs)
        self.ys = list(ys)
        self.yaws = list(yaws)
        self.frame_id = frame_id
        self.color = color
        self.visible = True
        # 미션의 어느 코스인지(main / start_left / ...). 라벨 스냅 거리는 자기 코스에
        # 대해서만 의미가 있으므로, 묶이지 않은 코스에는 거리를 표시하지 않습니다.
        self.mission_course = None

        # 전진/후진 표시는 로드 시점에 한 번만 정하고 편집으로는 바꾸지 않습니다
        # (geometry.recorded_flips 주석 참고).
        self.flips = geometry.recorded_flips(self.xs, self.ys, self.yaws)

        self.moved = 0
        self.inserted = 0
        self.deleted = 0
        self._undo = []
        self._redo = []
        self._saved_state = self._snapshot()

    # ------------------------------------------------------------------ 로드
    @classmethod
    def load(cls, path, color):
        course = formats.read_course(path)
        return cls(path, course.xs, course.ys, course.yaws, course.frame_id, color)

    @property
    def name(self):
        return os.path.basename(self.path)

    def __len__(self):
        return len(self.xs)

    # ------------------------------------------------------------------ 편집
    def _snapshot(self):
        return (tuple(self.xs), tuple(self.ys), tuple(self.yaws), tuple(self.flips),
                self.moved, self.inserted, self.deleted)

    def _restore(self, state):
        xs, ys, yaws, flips, moved, inserted, deleted = state
        self.xs, self.ys = list(xs), list(ys)
        self.yaws, self.flips = list(yaws), list(flips)
        self.moved, self.inserted, self.deleted = moved, inserted, deleted

    def _push_undo(self):
        self._undo.append(self._snapshot())
        self._redo.clear()

    def move_point(self, i, x, y):
        """i번 점을 (x, y)로. yaw는 이웃까지 다시 구하되 전진/후진은 보존합니다."""
        if not 0 <= i < len(self.xs):
            return
        self._push_undo()
        self.xs[i], self.ys[i] = float(x), float(y)
        self._refresh_yaw(geometry.touched_indices(i, len(self.xs)))
        self.moved += 1

    def insert_point(self, after, x, y):
        """after번 점 뒤에 새 점을 넣습니다. after=-1이면 맨 앞."""
        self._push_undo()
        i = after + 1
        before_idx = after if 0 <= after < len(self.xs) else None
        after_idx = i if i < len(self.xs) else None
        flip = geometry.inherited_flip(self.flips, before_idx, after_idx)
        self.xs.insert(i, float(x))
        self.ys.insert(i, float(y))
        self.yaws.insert(i, 0.0)
        self.flips.insert(i, flip)
        self._refresh_yaw(geometry.touched_indices(i, len(self.xs)))
        self.inserted += 1
        return i

    def delete_point(self, i):
        """i번 점을 지웁니다. 두 점 이하로는 줄이지 않습니다 -- 코스가 아니게 됩니다."""
        if not 0 <= i < len(self.xs) or len(self.xs) <= 2:
            return False
        self._push_undo()
        del self.xs[i], self.ys[i], self.yaws[i], self.flips[i]
        # 새로 이웃이 된 두 점의 yaw를 다시 구합니다.
        self._refresh_yaw([j for j in (i - 1, i) if 0 <= j < len(self.xs)])
        self.deleted += 1
        return True

    def set_flip(self, i, flip):
        """전진/후진 표시를 손으로 바꿉니다 -- 휴리스틱이 틀린 이음매용 탈출구."""
        if not 0 <= i < len(self.flips):
            return
        self._push_undo()
        self.flips[i] = 1 if flip >= 0 else -1
        self._refresh_yaw([i])

    def recompute_headings(self):
        """모든 점의 yaw를 경로에서 다시 구합니다. 전진/후진 표시는 보존합니다.

        왜 필요한가: 이 차의 절대 방위는 듀얼 RTK 안테나 기선에서만 나옵니다. 그게
        FIXED가 아닌 채로 녹화하면 EKF 헤딩에 절대 기준이 없어 자이로 적분만 남고,
        파일 전체가 거의 일정한 각도만큼 틀어집니다(측정: end_right.csv는 -35.7°,
        s_curve_right_angle.csv는 -27.9°). 위치는 멀쩡하므로 코스를 다시 딸 필요는
        없고, 전진 구간에서는 올바른 차체 헤딩이 곧 진행 방향입니다.

        주의: flips는 녹화된 yaw에서 뽑은 값이라, 틀어진 각이 90도에 가까우면 전진/
        후진 판정 자체가 못 믿을 값이 됩니다. 호출하는 쪽이 후진 점 개수를 사람에게
        보여 주고 확인을 받습니다.
        """
        self._push_undo()
        changed = 0
        largest = 0.0
        for i in range(len(self.xs)):
            new_yaw = geometry.yaw_after_edit(self.xs, self.ys, self.yaws, self.flips, i)
            delta = abs(geometry.normalize(new_yaw - self.yaws[i]))
            if delta > 1e-9:
                changed += 1
                largest = max(largest, delta)
            self.yaws[i] = new_yaw
        return changed, largest

    def heading_offset(self):
        """녹화된 yaw가 경로 방향에서 얼마나 틀어져 있는지(중앙값, 라디안).

        0에 가까우면 정상, 파일 전체가 일정하게 틀어져 있으면 그 값이 그대로 나옵니다.
        """
        import statistics
        deltas = []
        for i in range(len(self.xs)):
            heading = geometry.direction_at(self.xs, self.ys, i)
            if heading is None:
                continue
            reference = heading + (math.pi if self.flips[i] < 0 else 0.0)
            deltas.append(geometry.normalize(self.yaws[i] - reference))
        return statistics.median(deltas) if deltas else 0.0

    def _refresh_yaw(self, indices):
        for j in indices:
            self.yaws[j] = geometry.yaw_after_edit(
                self.xs, self.ys, self.yaws, self.flips, j)

    def undo(self):
        if not self._undo:
            return False
        self._redo.append(self._snapshot())
        self._restore(self._undo.pop())
        return True

    def redo(self):
        if not self._redo:
            return False
        self._undo.append(self._snapshot())
        self._restore(self._redo.pop())
        return True

    @property
    def can_undo(self):
        return bool(self._undo)

    @property
    def can_redo(self):
        return bool(self._redo)

    # ------------------------------------------------------------------ 저장
    @property
    def dirty(self):
        return self._snapshot() != self._saved_state

    def reverse_fraction(self, first, last):
        return geometry.reverse_fraction(self.xs, self.ys, self.yaws, first, last)

    def nearest(self, x, y):
        return geometry.nearest_index(self.xs, self.ys, x, y)

    def save(self, path=None):
        target = path or self.path
        formats.write_course(target, self.xs, self.ys, self.yaws, self.frame_id)
        self.path = target
        self._saved_state = self._snapshot()
        return target
