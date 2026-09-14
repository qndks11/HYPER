#!/usr/bin/env python3
# =====================================================================
# 순수 계산만 있습니다 -- Qt도 ROS도 파일도 건드리지 않습니다.
#
# 여기서 제일 중요한 것은 yaw 정책입니다. 점을 손으로 옮기면 녹화된 yaw가 더 이상
# 맞지 않는데, 이웃에서 진행 방향을 다시 구해 그대로 넣으면 후진으로 녹화한 구간이
# 조용히 망가집니다. 왜 그런지는 recorded_flip의 주석을 보세요.
# =====================================================================

import math

TWO_PI = 2.0 * math.pi

# 녹화 헤딩과 진행 방향이 이 값보다 덜 나란하면(|cos| < AMBIGUOUS_COS) 한 점만 보고
# 전진/후진을 판단하지 않습니다. 거의 겹친 점이나 방향 전환 이음매에서 부호가
# 뒤집히기 때문입니다.
AMBIGUOUS_COS = 0.25
# 그럴 때 다수결을 볼 창의 반폭(점 개수).
AMBIGUOUS_WINDOW = 10
# 이웃이 이보다 가까우면 진행 방향이 정의되지 않습니다.
DEGENERATE_M = 1e-6


def normalize(angle):
    """(-pi, pi]로 접습니다."""
    angle = math.fmod(angle + math.pi, TWO_PI)
    if angle <= 0.0:
        angle += TWO_PI
    return angle - math.pi


def direction_at(xs, ys, i):
    """i번 점의 진행 방향(중앙 차분, 양끝은 한쪽 차분). 정의 불가면 None."""
    n = len(xs)
    if n < 2:
        return None
    if 0 < i < n - 1:
        dx, dy = xs[i + 1] - xs[i - 1], ys[i + 1] - ys[i - 1]
    elif i == 0:
        dx, dy = xs[1] - xs[0], ys[1] - ys[0]
    else:
        dx, dy = xs[-1] - xs[-2], ys[-1] - ys[-2]
    if math.hypot(dx, dy) < DEGENERATE_M:
        return None
    return math.atan2(dy, dx)


def recorded_flip(xs, ys, yaws, i):
    """i번 점이 전진(+1)/후진(-1) 중 어느 쪽으로 녹화됐는지.

    CSV의 yaw는 EKF가 준 실제 차체 헤딩이라 전진/후진에 상관없이 그대로 씁니다
    (path_loader.hpp의 waypoint_heading 주석). 후진으로 녹화한 주차 구간에서는 이
    값이 진행 방향의 반대를 가리키고, 그게 맞습니다 -- nav2 RPP는 진행 방향을 포즈
    방향이 아니라 carrot의 차체 x부호로 판단하고, goal checker의 yaw 비교는 차체
    헤딩 기준이기 때문입니다.

    그래서 점을 옮길 때 이웃에서 구한 진행 방향을 그냥 넣으면 park_t_spot 같은
    후진 구간의 헤딩이 180도 뒤집혀, mission.yaml의 reverse: true가 거짓이 됩니다.
    판정식은 path_loader.hpp의 reverse_fraction과 같은 것을 씁니다 -- 그래야
    편집기와 로더가 같은 답을 냅니다.
    """
    heading = direction_at(xs, ys, i)
    if heading is None:
        return 1
    cos_between = math.cos(yaws[i] - heading)
    if abs(cos_between) >= AMBIGUOUS_COS:
        return 1 if cos_between >= 0.0 else -1

    # 한 점만으로는 애매합니다. 주변 창의 다수결을 봅니다.
    lo = max(0, i - AMBIGUOUS_WINDOW)
    hi = min(len(xs), i + AMBIGUOUS_WINDOW + 1)
    forward = backward = 0
    for j in range(lo, hi):
        h = direction_at(xs, ys, j)
        if h is None:
            continue
        c = math.cos(yaws[j] - h)
        if abs(c) < AMBIGUOUS_COS:
            continue
        if c >= 0.0:
            forward += 1
        else:
            backward += 1
    return -1 if backward > forward else 1


def recorded_flips(xs, ys, yaws):
    """로드 시점에 한 번 계산해 두는 점별 전진/후진 표시.

    편집된 좌표에서 다시 계산하지 않는 것이 요점입니다. 크게 끌어 놓은 한 번의
    드래그가 주차 지점의 의미를 스스로 뒤집어 버리면 안 됩니다.
    """
    return [recorded_flip(xs, ys, yaws, i) for i in range(len(xs))]


def yaw_after_edit(xs, ys, yaws, flips, i):
    """편집된 좌표에서 i번 점의 새 yaw. 전진/후진 표시는 보존합니다."""
    heading = direction_at(xs, ys, i)
    if heading is None:
        # 이웃이 겹쳐 진행 방향이 없습니다. 차체 헤딩을 굳이 바꿀 이유가 없습니다.
        return yaws[i]
    return normalize(heading + (math.pi if flips[i] < 0 else 0.0))


def touched_indices(i, n):
    """i를 옮기면 yaw를 다시 구해야 하는 점들. 이웃까지만이고 더 번지지 않습니다."""
    return [j for j in (i - 1, i, i + 1) if 0 <= j < n]


def reverse_fraction(xs, ys, yaws, first, last):
    """녹화된 헤딩이 진행 방향을 거스르는 표본의 비율.

    path_loader.hpp의 같은 이름 함수를 옮긴 것입니다. mission_loader가 로드 시점에
    reverse: true 구간이 정말 후진으로 녹화됐는지 검사할 때 쓰는 값이라, 저장하기
    전에 같은 검사를 여기서 먼저 돌려 볼 수 있습니다.
    """
    total = opposing = 0
    n = len(xs)
    for i in range(first, min(last, n - 1)):
        dx, dy = xs[i + 1] - xs[i], ys[i + 1] - ys[i]
        if math.hypot(dx, dy) < DEGENERATE_M:
            continue
        total += 1
        if math.cos(yaws[i]) * dx + math.sin(yaws[i]) * dy < 0.0:
            opposing += 1
    return 0.0 if total == 0 else opposing / total


def nearest_index(xs, ys, x, y):
    """(x, y)에 가장 가까운 점의 인덱스와 거리."""
    best, best_d2 = 0, float("inf")
    for i, (px, py) in enumerate(zip(xs, ys)):
        d2 = (px - x) ** 2 + (py - y) ** 2
        if d2 < best_d2:
            best, best_d2 = i, d2
    return best, math.sqrt(best_d2)


def inherited_flip(flips, before, after):
    """삽입된 점이 물려받을 전진/후진 표시.

    양쪽 이웃이 같으면 그 값입니다. 다르면 방향 전환 이음매 위에 넣은 것이므로,
    앞쪽 이웃을 따릅니다 -- 새 점은 그 구간의 끝을 잇는 쪽에 가깝습니다.
    """
    if before is None:
        return flips[after] if after is not None else 1
    if after is None:
        return flips[before]
    return flips[before] if flips[before] == flips[after] else flips[before]
