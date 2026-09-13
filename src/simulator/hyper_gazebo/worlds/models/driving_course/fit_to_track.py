#!/usr/bin/env python3
"""Gazebo 월드를 용인 트랙의 map 좌표계에 맞추는 닮음변환(similarity)을 구합니다.

왜 필요한가
-----------
sim의 코스(meshes/course.png)는 용인 트랙을 그대로 옮겨 그린 것입니다 --
교차로, 곡선(S)코스, 평행주차 베이, 가속코스, 출발/종료 표시가 항공사진
(meshes/real_course.png)과 하나씩 대응합니다. 다만 **돌아가 있고 ~11% 크게**
그려졌고, 월드의 <spherical_coordinates>도 서울시청(37.5665, 126.9780)으로
잡혀 있었습니다. 그래서 sim의 map 좌표와 실차의 map 좌표가 서로 달랐고,
웨이포인트와 mission.yaml을 양쪽에서 같이 쓸 수 없었습니다.

datums.yaml이 heading_deg를 0으로 못박아 map X=East, Y=North이고, gz navsat
센서는 월드 좌표에서 위경도를 만들어 냅니다. 즉 **sim에서는 map 프레임과
Gazebo 월드 프레임이 같습니다**(teleport_service.py가 이미 이 전제 위에 있습니다).
따라서 두 좌표계를 맞추는 일은 "월드 전체에 닮음변환 한 번 + datum 교체"로
끝납니다.

    p_map = s * R(theta) * p_world + t

구하는 방법
-----------
sim 주행 기록(sim.csv, sim1.csv)을 실차 기록(track/*.csv)에 trimmed ICP로
맞춥니다. 매 반복에서 잔차 상위 10%를 버리는데, sim 랩이 실제로 달리지 않은
분기(end_left, start_right 등)가 대응점 없이 남기 때문입니다.

주의: ICP 골짜기가 얕아서 시작점 하나로는 국소최적에 빠집니다. 실제로 라벨
적합(scale 0.8475)에서 바로 출발하면 0.881로 수렴하지만 전역 최적은 0.903이고,
0.881은 코스 양 끝에서 2 m 넘게 어긋납니다. 그래서 theta 55..85도 x scale
0.78..1.02 격자(651개 시작점)를 전부 돌려 제일 싼 해를 고르고, 그 해만 전체
점으로 다시 정련합니다. 이 중 3분의 1이 같은 해로 모이고 흩어짐이
scale +-0.0004, theta +-0.08도라서 전역 최적이 하나임을 스스로 확인합니다.

교차검증: 두 미션 yaml에 **이름이 같은** 라벨 6개(light_1..3, park_t_entry,
accel_start, accel_end)로 Umeyama 닮음변환을 따로 풀어 회전각을 비교합니다.
사람이 같은 지형지물을 보고 찍은 점들이라 ICP와는 독립적인 산출물이고, 둘의
회전각이 0.7도 안에서 맞습니다.

결과(2026-09 기준, 아래 EXPECTED에 박아 둠)
    s = 0.90320, theta = +69.8424 deg, t = (-12.1547, -6.0617) m
    잔차 mean 0.884 / median 0.715 / p90 1.919 m

잔차 0.7 m는 변환으로 없앨 수 없습니다 -- 그림이 실제 아스팔트보다 곡선
반경을 예쁘게 이상화했기 때문입니다. 닮음변환은 회전/등방 스케일/평행이동만
할 수 있으므로 모양 자체의 차이는 남습니다.

쓰는 법
-------
    ./fit_to_track.py                     # 변환을 구해 출력 + course.align.yaml 갱신
    ./fit_to_track.py --transform-world OLD.world NEW.world
    ./fit_to_track.py --apply-alignment   # 스튜디오에서 손으로 맞춘 course.align.yaml 반영

세 번째 모드는 ICP를 돌리지 않습니다. waypoint studio에서 course.png를 실차 기록
위에 눈으로 맞추고 "정렬 저장"하면 meshes/course.align.yaml(중심, 가로 폭,
회전)이 바뀝니다. 그 값을 새 변환으로 삼아 **지금** 월드의 모든 <pose>에 옛
변환 -> 새 변환의 차이만 먹이고, ground.obj/hill.obj/model.sdf/EXPECTED/
course.align.yaml을 다시 씁니다. 사이드카가 EXPECTED와 같으면 아무것도 하지
않으므로 두 번 돌려도 안전합니다.

두 번째 모드는 **변환 전** 월드를 읽어 변환 후 월드를 씁니다. 이미 변환된
월드에 다시 돌리면 두 번 적용됩니다. 커밋된 track.world는 이미 변환된
상태이므로, 다시 만들어야 하면 변환 전 원본을 git에서 꺼내 쓰세요:

    git show <변환-이전-커밋>:src/simulator/hyper_gazebo/worlds/track.world > /tmp/old.world
    ./fit_to_track.py --transform-world /tmp/old.world ../../track.world

웨이포인트를 다시 녹화해서 변환을 재산출했다면 아래 EXPECTED도 같이 고치고,
월드/메시/course.align.yaml을 전부 다시 만들어야 합니다(셋이 따로 놀면 map !=
world가 되어 전부 어긋납니다).
"""
import argparse
import csv
import glob
import os
import re
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..', '..', '..'))
WAYPOINTS = os.path.join(REPO, 'src', 'planning', 'hyper_waypoint', 'waypoints')
ALIGN_YAML = os.path.join(HERE, 'meshes', 'course.align.yaml')

# ground.obj 쿼드의 변환 전 반너비/반높이 (build_course.py가 만든 값).
QUAD_HALF_X = 51.25
QUAD_HALF_Y = 61.25

# 커밋된 값. 재산출 결과가 여기서 벗어나면 월드/메시도 같이 다시 만들어야 합니다.
EXPECTED = dict(scale=0.90320, theta_deg=69.8424, tx=-12.1547, ty=-6.0617)

# mission_sim.yaml과 mission_track.yaml에 같은 이름으로 있는 라벨.
# 사람이 같은 지형지물을 보고 찍었으므로 대응관계가 확실합니다.
SIM_LABELS = {
    'light_1': (12.202, -8.547),
    'light_2': (-0.500, 1.395),
    'light_3': (-9.691, -11.769),
    'park_t_entry': (-26.564, -21.848),
    'accel_start': (-36.231, 41.459),
    'accel_end': (-39.314, -34.026),
}
TRACK_LABELS = {
    'light_1': (-0.846, 1.740),
    'light_2': (-12.994, -6.010),
    'light_3': (-5.245, -19.289),
    'park_t_entry': (-2.297, -35.820),
    'accel_start': (-53.250, -26.930),
    'accel_end': (2.155, -49.931),
}


def load_xy(path):
    """웨이포인트 CSV에서 (x, y)만 뽑습니다.

    구형 23열 기록과 신형 5열 기록이 섞여 있어 열 이름으로 찾습니다
    (hyper_waypoint/README.md, path_loader.hpp와 같은 방식).
    """
    pts = []
    with open(path) as handle:
        for row in csv.DictReader(handle):
            pts.append((float(row['x']), float(row['y'])))
    return np.array(pts)


def umeyama(src, dst):
    """대응하는 두 점집합 사이의 닮음변환 (Umeyama 1991)을 닫힌 형태로 풉니다."""
    mean_src, mean_dst = src.mean(0), dst.mean(0)
    src0, dst0 = src - mean_src, dst - mean_dst
    cov = (dst0.T @ src0) / len(src)
    u, sv, vt = np.linalg.svd(cov)
    # 반사(거울상)가 나오지 않도록 det를 강제합니다 -- 코스는 뒤집히면 안 됩니다.
    d = np.diag([1.0, np.sign(np.linalg.det(u @ vt))])
    rot = u @ d @ vt
    scale = (sv @ np.diag(d)).sum() / ((src0 ** 2).sum() / len(src))
    return scale, rot, mean_dst - scale * rot @ mean_src


def rot2d(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def icp(src, dst, tree, scale, rot, trans, iters, trim=90):
    """trimmed ICP. 매 반복에서 잔차 상위 (100-trim)%를 버리고 다시 풉니다."""
    for _ in range(iters):
        dist, idx = tree.query(scale * (src @ rot.T) + trans)
        # sim 랩이 달리지 않은 분기(end_left, start_right 등)는 대응점이 없습니다.
        keep = dist < np.percentile(dist, trim)
        scale, rot, trans = umeyama(src[keep], dst[idx[keep]])
    return scale, rot, trans


def trimmed_cost(src, tree, scale, rot, trans, trim=90):
    """해들끼리 비교할 수 있게 같은 방식으로 계산한 적합 비용."""
    dist, _ = tree.query(scale * (src @ rot.T) + trans)
    return dist[dist < np.percentile(dist, trim)].mean()


def fit():
    """(scale, R, t)를 구합니다. 다중 시작점 -> 최적해 정련 순서입니다.

    ICP의 골짜기가 얕아서 시작점 하나로는 엉뚱한 국소최적에 빠집니다(라벨
    적합에서 바로 출발하면 scale 0.881로 수렴하는데, 전역 최적은 0.903입니다).
    그래서 theta x scale 격자를 쭉 훑어 각각 ICP를 돌리고 제일 싼 해를 고릅니다.
    """
    from scipy.spatial import cKDTree

    # --- 라벨 적합: 초기값이 아니라 독립적인 교차검증용입니다 ---
    keys = sorted(SIM_LABELS)
    lsrc = np.array([SIM_LABELS[k] for k in keys])
    ldst = np.array([TRACK_LABELS[k] for k in keys])
    lscale, lrot, ltrans = umeyama(lsrc, ldst)
    lres = np.linalg.norm(lscale * (lsrc @ lrot.T) + ltrans - ldst, axis=1)
    label_theta = np.degrees(np.arctan2(lrot[1, 0], lrot[0, 0]))
    print('라벨 %d개 독립 적합(교차검증용): scale %.4f  theta %+.3f deg  t (%+.3f, %+.3f)'
          % (len(keys), lscale, label_theta, *ltrans))
    print('  잔차 mean %.2f m  max %.2f m' % (lres.mean(), lres.max()))

    # sim 랩 2개만 씁니다. sim_left/sim_right는 짧은 분기라 적합을 끌어당깁니다.
    # 이 기록들은 변환 전 좌표계의 것이라 변환을 구한 뒤 트리에서 지웠습니다
    # (waypoints/simulation/). 다시 구해야 하면 git에서 꺼내 쓰세요 -- 어느 커밋에
    # 있었는지는 `git log --diff-filter=D -- src/planning/hyper_waypoint/waypoints/simulation`.
    try:
        src = np.vstack([load_xy(os.path.join(WAYPOINTS, 'simulation', name))
                         for name in ('sim.csv', 'sim1.csv')])
    except FileNotFoundError as exc:
        raise SystemExit(
            '변환 전 sim 주행 기록이 트리에 없습니다 (%s).\n'
            '이 기록들은 변환 전 좌표계의 것이라 좌표 통일 작업에서 지웠습니다.\n'
            '다시 적합하려면 git에서 되살리세요:\n'
            '    git log --diff-filter=D -- src/planning/hyper_waypoint/waypoints/simulation\n'
            '    git checkout <그 커밋>^ -- src/planning/hyper_waypoint/waypoints/simulation\n'
            % exc.filename) from exc
    dst = np.vstack([load_xy(p)
                     for p in sorted(glob.glob(os.path.join(WAYPOINTS, 'track', '*.csv')))])
    tree = cKDTree(dst)
    print('\n다중 시작점 ICP: sim %d점 -> track %d점' % (len(src), len(dst)))

    sweep = src[::5]                       # 훑을 때는 솎아 내고, 정련은 전체로 합니다
    dst_mean = dst.mean(0)
    sweep_mean = sweep.mean(0)
    sols = []
    for theta0 in np.arange(55.0, 86.0, 1.0):
        for scale0 in np.arange(0.78, 1.021, 0.01):
            rot0 = rot2d(np.radians(theta0))
            trans0 = dst_mean - scale0 * rot0 @ sweep_mean   # 무게중심을 맞춰 출발
            s, r, t = icp(sweep, dst, tree, scale0, rot0, trans0, iters=40)
            sols.append((trimmed_cost(sweep, tree, s, r, t), s, r, t))
    sols.sort(key=lambda row: row[0])

    # 전역 최적이 하나인지(= 이 변환을 믿어도 되는지) 스스로 확인합니다.
    best_cost = sols[0][0]
    near = [row for row in sols if row[0] < best_cost * 1.05]
    spread = np.array([[row[1], np.degrees(np.arctan2(row[2][1, 0], row[2][0, 0])),
                        row[3][0], row[3][1]] for row in near])
    print('  최적 근방(비용 5%% 이내)으로 수렴한 시작점 %d/%d개'
          % (len(near), len(sols)))
    print('    scale %.5f +- %.5f   theta %.3f +- %.3f deg'
          % (spread[:, 0].mean(), spread[:, 0].std(),
             spread[:, 1].mean(), spread[:, 1].std()))
    print('    t (%.3f +- %.3f, %.3f +- %.3f) m'
          % (spread[:, 2].mean(), spread[:, 2].std(),
             spread[:, 3].mean(), spread[:, 3].std()))

    scale, rot, trans = icp(src, dst, tree, sols[0][1], sols[0][2], sols[0][3], iters=200)
    dist, _ = tree.query(scale * (src @ rot.T) + trans)
    theta_deg = np.degrees(np.arctan2(rot[1, 0], rot[0, 0]))
    print('\n전체 점으로 정련한 최종 변환:')
    print('  scale      %.5f   (sim 코스가 %.1f%% 큼)' % (scale, (1 / scale - 1) * 100))
    print('  theta      %+.4f deg CCW  (= %.6f rad)' % (theta_deg, np.radians(theta_deg)))
    print('  t          (%+.4f, %+.4f) m' % (trans[0], trans[1]))
    print('  잔차       mean %.3f  median %.3f  p90 %.3f  p99 %.3f m'
          % (dist.mean(), np.median(dist), np.percentile(dist, 90), np.percentile(dist, 99)))
    print('  라벨 적합과의 회전 차이 %.2f deg (독립 산출물끼리의 교차검증)'
          % abs(theta_deg - label_theta))

    drift = max(abs(scale - EXPECTED['scale']) / EXPECTED['scale'],
                abs(theta_deg - EXPECTED['theta_deg']) / 360.0,
                abs(trans[0] - EXPECTED['tx']) / 100.0,
                abs(trans[1] - EXPECTED['ty']) / 100.0)
    if drift > 1e-3:
        print('\n*** 커밋된 EXPECTED와 다릅니다. 월드/메시/course.align.yaml을 모두\n'
              '*** 다시 만들고 EXPECTED도 갱신하세요 -- 셋 중 하나만 바뀌면 map != world입니다.',
              file=sys.stderr)
    return scale, rot, trans


def write_align_yaml(scale, theta_deg, trans):
    """waypoint studio가 배경 그림을 놓을 위치를 사이드카로 남깁니다.

    studio는 <이미지>.align.yaml을 formats.load_alignment로 읽습니다. 변환 뒤
    ground.obj 쿼드는 69.8도 돌아가 있어서 축정렬 bbox로는 그릴 수 없고, 이
    사이드카가 그 회전을 담습니다.
    """
    with open(ALIGN_YAML, 'w') as handle:
        handle.write('# fit_to_track.py가 생성합니다 -- 손으로 고치지 마세요.\n')
        handle.write('# Gazebo 월드의 course.png가 map 프레임에서 차지하는 자리입니다.\n')
        handle.write('# image: course.png\n')
        handle.write('center_x: %.4f\n' % trans[0])
        handle.write('center_y: %.4f\n' % trans[1])
        handle.write('width_m: %.4f\n' % (2 * QUAD_HALF_X * scale))
        handle.write('rotation_deg: %.4f\n' % theta_deg)
    print('\n%s 갱신' % os.path.relpath(ALIGN_YAML, REPO))


POSE_RE = re.compile(r'(<pose>)\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)'
                     r'\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s*(</pose>)')


def transform_world(src_path, dst_path, scale, rot, trans, theta):
    """변환 전 월드의 모든 <pose>에 닮음변환을 먹여 새 월드를 씁니다.

    x, y는 p_map = s*R*p + t로 옮기고 yaw에는 theta를 더합니다. z와 roll/pitch는
    그대로입니다 -- 소품은 실물 크기 그대로여야 하므로 **위치만** 옮기고 크기는
    건드리지 않습니다(코스가 10% 줄어도 라바콘은 라바콘 크기입니다).
    태양(<light>)의 pose는 방향만 의미가 있어 같이 옮겨도 무해합니다.
    """
    def repl(m):
        x, y, z = float(m.group(2)), float(m.group(3)), float(m.group(4))
        roll, pitch, yaw = float(m.group(5)), float(m.group(6)), float(m.group(7))
        nx, ny = scale * (rot @ np.array([x, y])) + trans
        return '%s%.4f %.4f %.4f %s %s %.5f%s' % (
            m.group(1), nx, ny, z, m.group(5), m.group(6), yaw + theta, m.group(8))

    with open(src_path) as handle:
        text = handle.read()
    out, count = POSE_RE.subn(repl, text)
    with open(dst_path, 'w') as handle:
        handle.write(out)
    print('\n%s -> %s  (<pose> %d개 변환)'
          % (os.path.relpath(src_path, REPO), os.path.relpath(dst_path, REPO), count))


def report_derived(scale, rot, trans, theta):
    """월드/메시/런치에 손으로 박아야 하는 값들을 뽑아 줍니다."""
    print('\n--- 반영해야 하는 값 ---')
    print('driving_course include pose : %.4f %.4f 0 0 0 %.6f' % (trans[0], trans[1], theta))
    print('ground.obj 쿼드 반너비/반높이: %.4f, %.4f  (원래 %.2f, %.2f)'
          % (QUAD_HALF_X * scale, QUAD_HALF_Y * scale, QUAD_HALF_X, QUAD_HALF_Y))
    print('ground collision <plane><size>: %.2f %.2f'
          % (2 * QUAD_HALF_X * scale, 2 * QUAD_HALF_Y * scale))
    print('차선 폭                       : 3.00 m -> %.2f m' % (3.0 * scale))
    start = load_xy(os.path.join(WAYPOINTS, 'track', 'start_left.csv'))[0]
    print('스폰(track/start_left.csv #0) : x %.4f  y %.4f' % (start[0], start[1]))


WORLD = os.path.abspath(os.path.join(HERE, '..', '..', 'track.world'))
MODEL_SDF = os.path.join(HERE, 'model.sdf')
GROUND_OBJ = os.path.join(HERE, 'meshes', 'ground.obj')
BUILD_HILL = os.path.join(HERE, 'build_hill.py')
BUILD_COURSE = os.path.join(HERE, 'build_course.py')


def read_alignment(path):
    """course.align.yaml의 key: value 줄만 읽습니다 (주석 무시)."""
    doc = {}
    with open(path) as handle:
        for line in handle:
            line = line.split('#', 1)[0].strip()
            if ':' in line:
                key, value = line.split(':', 1)
                doc[key.strip()] = float(value)
    return doc


def _sub(text, pattern, repl, where):
    out, count = re.subn(pattern, repl, text, flags=re.M)
    if count == 0:
        raise SystemExit('%s: 바꿀 자리를 못 찾았습니다 (%s)' % (where, pattern))
    return out


def apply_alignment(align_path):
    """스튜디오에서 맞춘 사이드카를 새 닮음변환으로 월드/메시에 반영합니다.

    옛 변환 p = s R(th) p0 + t, 새 변환 p' = s' R(th') p0 + t'이므로 이미 변환된
    월드 좌표에는 p' = (s'/s) R(th'-th) (p - t) + t'만 먹이면 됩니다. 변환 전 원본
    월드를 git에서 꺼낼 필요가 없고, 그 사이 손으로 옮긴 소품도 그대로 따라갑니다.
    """
    doc = read_alignment(align_path)
    new_scale = doc['width_m'] / (2 * QUAD_HALF_X)
    new_theta_deg = doc['rotation_deg']
    new_trans = np.array([doc['center_x'], doc['center_y']])
    old_scale = EXPECTED['scale']
    old_theta_deg = EXPECTED['theta_deg']
    old_trans = np.array([EXPECTED['tx'], EXPECTED['ty']])

    if (abs(new_scale - old_scale) < 1e-5 and abs(new_theta_deg - old_theta_deg) < 1e-4
            and np.linalg.norm(new_trans - old_trans) < 1e-4):
        raise SystemExit('%s가 EXPECTED와 같습니다 -- 이미 반영돼 있습니다.'
                         % os.path.relpath(align_path, REPO))

    ratio = new_scale / old_scale
    dtheta = np.radians(new_theta_deg - old_theta_deg)
    drot = rot2d(dtheta)
    print('옛 변환 : scale %.5f  theta %+.4f deg  t (%+.4f, %+.4f)'
          % (old_scale, old_theta_deg, *old_trans))
    print('새 변환 : scale %.5f  theta %+.4f deg  t (%+.4f, %+.4f)'
          % (new_scale, new_theta_deg, *new_trans))
    print('차이    : 크기 %+.2f%%  회전 %+.3f deg  이동 (%+.3f, %+.3f) m'
          % ((ratio - 1) * 100, np.degrees(dtheta), *(new_trans - old_trans)))
    print('차선 폭 : %.2f m -> %.2f m' % (3.0 * old_scale, 3.0 * new_scale))

    # 전부 메모리에서 만들고 검사한 뒤에 씁니다 -- 중간에 실패해 반만 바뀐 상태가
    # 남으면 map != world가 됩니다.
    edits = {}

    def move_pose(m):
        x, y = float(m.group(2)), float(m.group(3))
        nx, ny = ratio * (drot @ (np.array([x, y]) - old_trans)) + new_trans
        return '%s%.4f %.4f %s %s %s %.5f%s' % (
            m.group(1), nx, ny, m.group(4), m.group(5), m.group(6),
            float(m.group(7)) + dtheta, m.group(8))

    with open(WORLD) as handle:
        world = handle.read()
    world, pose_count = POSE_RE.subn(move_pose, world)
    world = _sub(world, r'등방 스케일\s+\d\.\d+', lambda m: m.group(0).replace(
        re.search(r'\d\.\d+', m.group(0)).group(0), '%.5f' % new_scale), 'track.world')
    edits[WORLD] = world

    def scale_vertex(m):
        x, y = float(m.group(1)), float(m.group(2))
        return 'v %.4f %.4f %s' % (np.sign(x) * QUAD_HALF_X * new_scale,
                                  np.sign(y) * QUAD_HALF_Y * new_scale, m.group(3))

    with open(GROUND_OBJ) as handle:
        edits[GROUND_OBJ] = _sub(handle.read(), r'^v (\S+) (\S+) (\S+)$',
                                 scale_vertex, 'ground.obj')

    width, height = 2 * QUAD_HALF_X * new_scale, 2 * QUAD_HALF_Y * new_scale
    with open(MODEL_SDF) as handle:
        sdf = handle.read()
    sdf = _sub(sdf, r'<size>[\d.]+ [\d.]+</size>',
               '<size>%.2f %.2f</size>' % (width, height), 'model.sdf')
    sdf = _sub(sdf, r'[\d.]+ x [\d.]+ m, [\d.]+ m lanes',
               '%.2f x %.2f m, %.2f m lanes' % (width, height, 3.0 * new_scale), 'model.sdf')
    sdf = _sub(sdf, r'등방 스케일 \d\.\d+', '등방 스케일 %.5f' % new_scale, 'model.sdf')
    edits[MODEL_SDF] = sdf

    with open(BUILD_HILL) as handle:
        edits[BUILD_HILL] = _sub(handle.read(), r'^SCALE = [\d.]+$',
                                 'SCALE = %.5f' % new_scale, 'build_hill.py')
    with open(BUILD_COURSE) as handle:
        edits[BUILD_COURSE] = _sub(handle.read(), r'등방 스케일\n?(#\s*)?\d\.\d+',
                                   lambda m: re.sub(r'\d\.\d+', '%.5f' % new_scale,
                                                    m.group(0)), 'build_course.py')

    this = os.path.abspath(__file__)
    with open(this) as handle:
        edits[this] = _sub(handle.read(), r'^EXPECTED = dict\(.*\)$',
                           'EXPECTED = dict(scale=%.5f, theta_deg=%.4f, tx=%.4f, ty=%.4f)'
                           % (new_scale, new_theta_deg, *new_trans), 'fit_to_track.py')

    for path, text in edits.items():
        with open(path, 'w') as handle:
            handle.write(text)
        print('%s 갱신' % os.path.relpath(path, REPO))
    print('  (track.world <pose> %d개 이동)' % pose_count)

    subprocess.run([sys.executable, BUILD_HILL], cwd=HERE, check=True)
    print('meshes/hill.obj 재생성')
    write_align_yaml(new_scale, new_theta_deg, new_trans)
    report_derived(new_scale, rot2d(np.radians(new_theta_deg)), new_trans,
                   np.radians(new_theta_deg))
    print('\nhyper_gazebo/README.md의 변환 표와 차선 폭 문장은 손으로 고치세요.')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--transform-world', nargs=2, metavar=('OLD', 'NEW'),
                        help='변환 전 월드를 읽어 변환 후 월드를 씁니다 (두 번 돌리지 마세요)')
    parser.add_argument('--apply-alignment', nargs='?', const=ALIGN_YAML, metavar='YAML',
                        help='스튜디오에서 맞춘 사이드카를 월드/메시에 반영합니다 (ICP 없음)')
    args = parser.parse_args()

    if args.apply_alignment:
        apply_alignment(args.apply_alignment)
        return

    scale, rot, trans = fit()
    theta = np.arctan2(rot[1, 0], rot[0, 0])
    write_align_yaml(scale, np.degrees(theta), trans)
    report_derived(scale, rot, trans, theta)
    if args.transform_world:
        transform_world(args.transform_world[0], args.transform_world[1],
                        scale, rot, trans, theta)


if __name__ == '__main__':
    main()
