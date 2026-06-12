"""ブラウザ上のスライダーで6軸を動かせるロボットアームビューア (viser)。

使い方:
  uv run python arm_viewer_viser.py
  → http://127.0.0.1:8080 をブラウザで開き、右上のJ1〜J6スライダーを操作

メッシュは起動時に一度だけテッセレートし、以後は関節フレームの回転だけを
更新するのでスライダー操作はリアルタイムに反映される。
(干渉チェックはしないので、当たりの確認は view_arm.py / robot_arm.py 側で)
"""

import math

import numpy as np
import viser

import robot_arm as ra


def loc_to_pq(loc):
    """build123d Location -> (position, wxyz quaternion)."""
    t = loc.wrapped.Transformation()
    q = t.GetRotation()
    tp = t.TranslationPart()
    return (np.array([tp.X(), tp.Y(), tp.Z()]),
            np.array([q.W(), q.X(), q.Y(), q.Z()]))


def mesh_arrays(shape, tol=0.3):
    v, f = shape.tessellate(tol)
    return (np.array([(p.X, p.Y, p.Z) for p in v], dtype=np.float32),
            np.array(f, dtype=np.uint32))


print("tessellating parts (initial run takes ~1 min)...")
servo_mesh = mesh_arrays(ra.servo_raw.moved(ra.SHIFT))
bracket_meshes = {n: mesh_arrays(p) for n, p in ra.bracket_parts().items()}

ORANGE = (240, 140, 26)
GRAY = (70, 70, 78)
DARK = (45, 45, 45)

server = viser.ViserServer(host="127.0.0.1", port=8080)
server.scene.add_grid("/ground", width=500, height=500, plane="xy")

server.scene.add_mesh_simple("/base", *bracket_meshes["base"], color=DARK)
p, q = loc_to_pq(ra.L1)
server.scene.add_frame("/s1", position=p, wxyz=q, show_axes=False)
server.scene.add_mesh_simple("/s1/servo", *servo_mesh, color=GRAY)

# kinematic chain: /s1/j1/(b1, s2/j2/(b2, s3/...))   j_i = joint rotation
joint_frames = []
path = "/s1"
for i, name in enumerate(ra.BRACKET_NAMES):
    path += f"/j{i + 1}"
    joint_frames.append(server.scene.add_frame(path, show_axes=False))
    server.scene.add_mesh_simple(f"{path}/{name}", *bracket_meshes[name],
                                 color=ORANGE)
    if i < 5:
        p, q = loc_to_pq(ra.CHAIN[i])
        path += "/s"
        server.scene.add_frame(path, position=p, wxyz=q, show_axes=False)
        server.scene.add_mesh_simple(f"{path}/servo", *servo_mesh, color=GRAY)

sliders = []
for i in range(6):
    lim = 180 if i in (0, 3, 5) else 100  # web/tail interference limits
    sliders.append(server.gui.add_slider(
        f"J{i + 1} [deg]", min=-lim, max=lim, step=1, initial_value=0))


def update(_=None):
    for s, jf in zip(sliders, joint_frames):
        half = math.radians(s.value) / 2
        jf.wxyz = np.array([math.cos(half), 0.0, 0.0, math.sin(half)])


for s in sliders:
    s.on_update(update)

reset = server.gui.add_button("reset pose")


@reset.on_click
def _(_):
    for s in sliders:
        s.value = 0


print("open http://127.0.0.1:8080 in your browser (Ctrl+C to quit)")
server.sleep_forever()
