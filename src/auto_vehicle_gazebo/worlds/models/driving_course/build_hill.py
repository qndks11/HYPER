"""
Generates meshes/hill.obj (+ hill.mtl): a smooth hill overlaid on the flat
ground, spanning the "blue" (uphill) + "red" (downhill) rectangles marked in
meshes/elevation.png.

  - Profile along the direction of travel (Y): smoothstep up from 0 -> peak
    over the blue span, then smoothstep down from peak -> 0 over the red
    span. Zero slope at both the base and the peak, so it blends tangentially
    into the flat ground and has a rounded (not pointy) hilltop.
  - Profile across the road (X): constant at any given Y (no lateral taper).
    The two long edges are vertical walls dropping straight to the
    surrounding ground level ("steep walls" on the sides).
  - Top surface is textured with course.png using the exact same world->UV
    mapping as meshes/ground.obj, so it shows the same road markings that
    are already painted there, just lifted into 3D.

Run from this directory: python3 build_hill.py
"""
import numpy as np

# ---------- ground quad extents (must match meshes/ground.obj) ----------
Xh, Yh = 51.25, 61.25  # world half-extents (m)

# ---------- hill footprint, derived from elevation.png red/blue boxes ----------
# (see conversation / git history for the pixel->world derivation)
RED = dict(xmin=37.243, xmax=45.626, ymin=1.200, ymax=25.007)
BLUE = dict(xmin=37.712, xmax=46.095, ymin=-16.472, ymax=1.309)

X_MIN = min(RED['xmin'], BLUE['xmin'])
X_MAX = max(RED['xmax'], BLUE['xmax'])
Y_START = BLUE['ymin']                       # base of the climb (height 0)
Y_PEAK = (BLUE['ymax'] + RED['ymin']) / 2.0  # blue/red seam (height = PEAK_H)
Y_END = RED['ymax']                          # base of the descent (height 0)
PEAK_H = 1.5

N = 140  # samples along Y


def smoothstep(t):
    t = np.clip(t, 0.0, 1.0)
    return 3 * t**2 - 2 * t**3


def height(y):
    y = np.asarray(y, dtype=np.float64)
    h = np.zeros_like(y)
    rising = y <= Y_PEAK
    t_up = (y[rising] - Y_START) / (Y_PEAK - Y_START)
    h[rising] = PEAK_H * smoothstep(t_up)
    falling = ~rising
    t_down = (y[falling] - Y_PEAK) / (Y_END - Y_PEAK)
    h[falling] = PEAK_H * (1.0 - smoothstep(t_down))
    return h


def world_to_uv(x, y):
    u = (x + Xh) / (2 * Xh)
    v = (y + Yh) / (2 * Yh)
    return u, v


ys = np.linspace(Y_START, Y_END, N)
hs = height(ys)

verts = []       # list of (x,y,z)
uvs = []         # list of (u,v)
normals = []     # list of (nx,ny,nz)
top_faces = []   # (vi,vti,ni) triples per corner, 3 corners per face
wall_faces = []  # (vi,ni) pairs per corner (no texture), 3 corners per face


def add_vert(x, y, z):
    verts.append((x, y, z))
    return len(verts)  # 1-indexed


def add_uv(u, v):
    uvs.append((u, v))
    return len(uvs)


def add_normal(nx, ny, nz):
    n = np.array([nx, ny, nz], dtype=np.float64)
    ln = np.linalg.norm(n)
    n = n / ln if ln > 1e-12 else np.array([0.0, 0.0, 1.0])
    normals.append(tuple(n))
    return len(normals)


def oriented_tri(corners, desired_dir):
    """corners: list of 3 (vertex_id, ...) tuples, vertex_id is 1-indexed into
    `verts`. Reorders (if needed) so the triangle's winding faces
    `desired_dir`, verified via the actual cross product -- this is what the
    renderer's backface culling keys off, not the vn we write out."""
    p = [np.array(verts[c[0] - 1]) for c in corners]
    cross = np.cross(p[1] - p[0], p[2] - p[0])
    if np.dot(cross, desired_dir) < 0:
        return (corners[0], corners[2], corners[1])
    return tuple(corners)


# ---------- top surface: ruled strip between the two long edges ----------
n_up = add_normal(0, 0, 1)  # approximate; slope is gentle enough that a shared
                             # near-vertical normal looks fine on a smooth hill
left_top_v = [add_vert(X_MIN, y, h) for y, h in zip(ys, hs)]
right_top_v = [add_vert(X_MAX, y, h) for y, h in zip(ys, hs)]
left_top_uv = [add_uv(*world_to_uv(X_MIN, y)) for y in ys]
right_top_uv = [add_uv(*world_to_uv(X_MAX, y)) for y in ys]

UP = np.array([0.0, 0.0, 1.0])
for i in range(N - 1):
    a = (left_top_v[i], left_top_uv[i], n_up)
    b = (right_top_v[i], right_top_uv[i], n_up)
    c = (right_top_v[i + 1], right_top_uv[i + 1], n_up)
    d = (left_top_v[i + 1], left_top_uv[i + 1], n_up)
    top_faces.append(oriented_tri((a, b, c), UP))
    top_faces.append(oriented_tri((a, c, d), UP))

# ---------- side walls: vertical strips dropping to z=0 ----------
n_left = add_normal(-1, 0, 0)
n_right = add_normal(1, 0, 0)

left_bot_v = [add_vert(X_MIN, y, 0.0) for y in ys]
right_bot_v = [add_vert(X_MAX, y, 0.0) for y in ys]

OUT_LEFT = np.array([-1.0, 0.0, 0.0])
OUT_RIGHT = np.array([1.0, 0.0, 0.0])
for i in range(N - 1):
    # left wall (x=X_MIN): outward normal is -X
    bt = (left_top_v[i], n_left)
    tt = (left_top_v[i + 1], n_left)
    bb = (left_bot_v[i], n_left)
    tb = (left_bot_v[i + 1], n_left)
    wall_faces.append(oriented_tri((bb, tb, tt), OUT_LEFT))
    wall_faces.append(oriented_tri((bb, tt, bt), OUT_LEFT))

    # right wall (x=X_MAX): outward normal is +X
    bt = (right_top_v[i], n_right)
    tt = (right_top_v[i + 1], n_right)
    bb = (right_bot_v[i], n_right)
    tb = (right_bot_v[i + 1], n_right)
    wall_faces.append(oriented_tri((bb, tt, tb), OUT_RIGHT))
    wall_faces.append(oriented_tri((bb, bt, tt), OUT_RIGHT))

# ---------- write OBJ + MTL ----------
with open('meshes/hill.mtl', 'w') as f:
    f.write('newmtl course\nKa 1 1 1\nKd 1 1 1\nd 1\nillum 1\nmap_Kd course.png\n')
    f.write('newmtl wall\nKa 0.35 0.35 0.35\nKd 0.35 0.35 0.35\nd 1\nillum 1\n')

with open('meshes/hill.obj', 'w') as f:
    f.write('mtllib hill.mtl\n')
    for x, y, z in verts:
        f.write(f'v {x:.4f} {y:.4f} {z:.4f}\n')
    for u, v in uvs:
        f.write(f'vt {u:.6f} {v:.6f}\n')
    for nx, ny, nz in normals:
        f.write(f'vn {nx:.4f} {ny:.4f} {nz:.4f}\n')

    f.write('usemtl course\n')
    for (v0, vt0, n0), (v1, vt1, n1), (v2, vt2, n2) in top_faces:
        f.write(f'f {v0}/{vt0}/{n0} {v1}/{vt1}/{n1} {v2}/{vt2}/{n2}\n')

    f.write('usemtl wall\n')
    for (v0, n0), (v1, n1), (v2, n2) in wall_faces:
        f.write(f'f {v0}//{n0} {v1}//{n1} {v2}//{n2}\n')

print(f'hill footprint: X[{X_MIN:.2f},{X_MAX:.2f}] Y[{Y_START:.2f},{Y_END:.2f}] peak {PEAK_H}m @ Y={Y_PEAK:.2f}')
print(f'verts={len(verts)} uvs={len(uvs)} normals={len(normals)} '
      f'top_tris={len(top_faces)} wall_tris={len(wall_faces)}')
