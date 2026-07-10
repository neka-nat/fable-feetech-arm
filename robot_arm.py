"""6-axis robot arm built around the Feetech STS3215 servo (STS3215_03a.step).

All bracket interfaces are derived from dimensions measured directly on the
imported STEP model.  Servo local frame used below ("axis frame"):
  - origin on the output axis (STEP model shifted by x -= 12.5)
  - +Z along the output axis toward the drive wheel

Measured servo geometry (axis frame, mm):
  drive wheel  : Ø20 disc, face z=+18.7, 4 x Ø2.5 (M3 tap) on a 9.9 mm square
  center screw : head Ø~5.4 protruding to z=+20.2
  idler wheel  : Ø20 disc, face z=-17.7, same hole pattern,
                 center boss Ø6 protruding to z=-18.3
  top tabs     : surface z=+15.9, Ø1.5 pilot holes (M2 self-tapping) at
                 (-29.0, ±10.25) and (-8.3, ±10.25)
  bottom tabs  : surface z=-15.9, holes at (-32.8, ±10.25) and (-8.3, ±10.25)
  body         : x in [-35.2, 10.2], y ±12.4, case z ±14.4
  top ridge    : x in [-34.78, -8.51], y ±7.0, protrudes to z=+17.0
  connector bay: block x in [-29.9, -16.6], y ±9.2, bottom z=-17.7.
                 Two 3-pin headers, pins Ø2 pointing straight down (-Z) at
                 x [-15.05, -13.05], y ±2.8/±5.2/±7.6, tips z=-19.4.
                 Mated plugs slide on from below: plug bodies occupy about
                 x [-17, -11], y ±9.5, down to z ~ -26.5, wires exiting
                 further down/backward.  Brackets must keep this volume --
                 and the annulus it sweeps around the joint -- clear.
"""

import copy
from build123d import *

MM = 1.0

# ---------------------------------------------------------------- measured --
WHEEL_FACE = 18.7
IDLER_FACE = -17.7
TAB_TOP = 15.9
TAB_BOT = -15.9
WHEEL_R = 10.0
SCREW_HEAD_TOP = 20.2  # wheel center screw head
IDLER_BOSS_BOT = -18.3

HORN_HOLES = [(4.95, 4.95), (4.95, -4.95), (-4.95, 4.95), (-4.95, -4.95)]
TOP_TABS = [(-29.0, 10.25), (-29.0, -10.25), (-8.3, 10.25), (-8.3, -10.25)]
BOT_TABS = [(-32.8, 10.25), (-32.8, -10.25), (-8.3, 10.25), (-8.3, -10.25)]

BODY_X0, BODY_X1 = -35.2, 10.2
BODY_Y = 12.4
RIDGE_X0, RIDGE_X1 = -34.78, -8.51
CONN_X0, CONN_X1 = -29.9, -13.05

# ------------------------------------------------------------- clearances --
M3_CLEAR = 3.4 / 2          # wheel screws (servo holes are M3 tap size Ø2.5)
M2_CLEAR = 2.4 / 2          # tab screws (M2 self tapping into Ø1.5 pilots)
WHEEL_HOLE_R = 23.6 / 2     # bracket clearance hole around wheel/idler
IDLER_HOLE_R = 24.0 / 2
HEAD_RECESS_R = 9.5 / 2     # recess over the wheel center screw head
BOSS_RECESS_R = 7.0 / 2     # recess over the idler center boss
ACCESS_R = 6.0 / 2          # screwdriver access holes for M3 wheel screws

FIT_AX = 0.35               # print fit: gap per side, wheel/idler grip bosses
FIT_TAB = 0.2               # print fit: gap per side, tab clamp faces

PLATE_T = 4.0               # structural plate thickness
CLAMP_T = 2.5               # tab clamp plate thickness (limited by wheel face)
NARROW = 19.5               # U-bracket inner face when gripping a bare servo
WIDE = 24.25                # ... when gripping a servo held by U-link pads

CLAMP_DZ = 64.0             # axis offset gripped servo -> clamped servo
DECK_L = 54.0               # u_link length when ending in a roll-servo deck
ARM_L = 110.0               # shoulder->elbow link length
BASE_H = 30.0               # base servo axis frame height over base plate top


def box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(
        abs(x1 - x0), abs(y1 - y0), abs(z1 - z0)
    )


def cyl_z(r, z0, z1, x=0.0, y=0.0):
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(r, abs(z1 - z0))


def cyl_y(r, y0, y1, x=0.0, z=0.0):
    return Pos(x, (y0 + y1) / 2, z) * Rot(90, 0, 0) * Cylinder(r, abs(y1 - y0))


def cyl_x(r, x0, x1, y=0.0, z=0.0):
    return Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(r, abs(x1 - x0))


def csk_z(x, y, z_surf, up=True):
    """M2 countersink opening at the plane z=z_surf (head toward +z if up)."""
    s = 1 if up else -1
    hole = cyl_z(M2_CLEAR, z_surf - s * 12, z_surf + s * 2, x, y)
    cone = Pos(x, y, z_surf - s * 0.5) * Rot(0 if up else 180, 0, 0) * Cone(
        bottom_radius=1.2, top_radius=2.3, height=1.1
    )
    return hole + cone


def csk_x(y, z, x_surf, toward_pos_x=True):
    s = 1 if toward_pos_x else -1
    hole = cyl_x(M2_CLEAR, x_surf - s * 12, x_surf + s * 2, y, z)
    cone = Pos(x_surf - s * 0.5, y, z) * Rot(0, 90 if toward_pos_x else -90, 0) * Cone(
        bottom_radius=1.2, top_radius=2.3, height=1.1
    )
    return hole + cone


def csk_y(x, z, y_surf, toward_pos_y=True):
    s = 1 if toward_pos_y else -1
    hole = cyl_y(M2_CLEAR, y_surf - s * 12, y_surf + s * 2, x, z)
    cone = Pos(x, y_surf - s * 0.5, z) * Rot(-90 if toward_pos_y else 90, 0, 0) * Cone(
        bottom_radius=1.2, top_radius=2.3, height=1.1
    )
    return hole + cone


# ------------------------------------------------------------------ parts --
def horn_disc(access_holes=False):
    """Ø20 disc bolted onto the drive wheel (built on the gripped servo)."""
    p = cyl_z(WHEEL_R, WHEEL_FACE, WHEEL_FACE + PLATE_T)
    p -= cyl_z(HEAD_RECESS_R, WHEEL_FACE - 0.1, SCREW_HEAD_TOP + 0.3)
    for hx, hy in HORN_HOLES:
        p -= cyl_z(M3_CLEAR, WHEEL_FACE - 1, WHEEL_FACE + PLATE_T + 1, hx, hy)
    return p


def clamp_mount():
    """B1/B4: horn disc + floor slab + twin plates clamping the next servo.

    Built in the *gripped* servo's axis frame.  The clamped servo sits at
    Pos(0,0,CLAMP_DZ) with its axis along +Y (local), tail pointing down.
    """
    zc = CLAMP_DZ
    tabf = TAB_TOP + FIT_TAB                   # clamp faces sit FIT_TAB clear
    disc_top = WHEEL_FACE + PLATE_T            # 22.7
    slab_top = disc_top + 5.3                  # 28.0
    p = horn_disc()
    p += box(-16, 16, -(tabf + CLAMP_T), tabf + CLAMP_T, disc_top, slab_top)
    for hx, hy in HORN_HOLES:                  # M3 driver access through slab
        p -= cyl_z(ACCESS_R, disc_top - 1, slab_top + 1, hx, hy)

    # plate A (+y): clamps the next servo's top tabs, wheel pokes through
    pa = box(-16, 16, tabf, tabf + CLAMP_T, disc_top + 0.1, zc)
    pa += cyl_y(16, tabf, tabf + CLAMP_T, x=0, z=zc)
    pa -= cyl_y(WHEEL_HOLE_R, tabf - 1, tabf + CLAMP_T + 1, x=0, z=zc)
    pa -= box(-7.5, 7.5, tabf - 1, TAB_TOP + 1.45,  # case top ridge window
              zc + RIDGE_X0 - 1.5, zc + RIDGE_X1 + 1.5)
    for tx, ty in TOP_TABS:
        pa -= csk_y(x=ty, z=zc + tx, y_surf=tabf + CLAMP_T, toward_pos_y=True)

    # plate B (-y): clamps bottom tabs; idler pokes through.  The connector
    # window passes the mated plugs (they exit down-forward past the plate).
    pb = box(-16, 16, -tabf - CLAMP_T, -tabf, disc_top + 0.1, zc)
    pb += cyl_y(16, -tabf - CLAMP_T, -tabf, x=0, z=zc)
    pb -= cyl_y(IDLER_HOLE_R, -tabf - CLAMP_T - 1, -tabf + 1, x=0, z=zc)
    pb -= box(-9.7, 9.7, -tabf - CLAMP_T - 1, -TAB_TOP + 1,  # connector bay
              zc + CONN_X0 - 1.5, zc + CONN_X1 + 3.0)
    for tx, ty in BOT_TABS:
        pb -= csk_y(x=ty, z=zc + tx, y_surf=-tabf - CLAMP_T, toward_pos_y=False)

    return p + pa + pb


def u_link(length, inner, end):
    """U-bracket gripping wheel+idler of a servo, built in its axis frame.

    The link runs along local +X.  end='pads' clamps the next servo (same
    orientation, axis at Pos(length,0,0)) between internal pads; end='deck'
    carries a deck plate at x=length holding the next servo with its output
    axis along local +X (tail toward +y).
    """
    outer = inner + PLATE_T
    tabf = TAB_TOP + FIT_TAB
    x_end = length + (13.5 if end == "pads" else 2.5)

    def side_plate(s):  # s=+1 wheel side, s=-1 idler side
        if s > 0:
            pl = box(0, x_end, -16, 16, s * inner, s * outer)
            pl += cyl_z(16, s * inner, s * outer)
        else:
            # the gripped servo's mated plugs hang straight down over the
            # pins (r ~11..19 from the axis, down to z ~ -26.5).  Keep the
            # joint disc r10.25 and the first 22 mm of plate only 20.5 mm
            # wide so the rotating link sweeps past the plugs (~±95 deg).
            pl = box(0, 22, -10.25, 10.25, s * inner, s * outer)
            pl += box(20, x_end, -16, 16, s * inner, s * outer)
            pl += cyl_z(10.25, s * inner, s * outer)
        if end == "pads":
            pl += cyl_z(16, s * inner, s * outer, x=length)
        # contact boss FIT_AX short of the wheel / idler face; the M3 screws
        # pull it snug, and assembly gets 2*FIT_AX of insertion clearance
        if s > 0:
            pl += cyl_z(WHEEL_R, WHEEL_FACE + FIT_AX, inner + 0.1)
            pl -= cyl_z(HEAD_RECESS_R, WHEEL_FACE - 0.1,
                        SCREW_HEAD_TOP + 0.3 + FIT_AX)
        else:
            pl += cyl_z(WHEEL_R, -inner - 0.1, IDLER_FACE - FIT_AX)
            pl -= cyl_z(BOSS_RECESS_R, IDLER_FACE + 0.1,
                        IDLER_BOSS_BOT - 0.1 - FIT_AX)
        for hx, hy in HORN_HOLES:
            pl -= cyl_z(M3_CLEAR, s * (WHEEL_FACE - 2), s * (outer + 1), hx, hy)
        return pl

    # back web (-y side) first: it touches both side plates, keeping the
    # accumulated result a single connected solid
    web_x1 = x_end if end == "pads" else length - 2
    p = box(25, web_x1, -20, -16, -outer, outer)
    p += side_plate(+1)
    p += side_plate(-1)

    if end == "pads":
        # bottom tabs: only the tail pair -- the front pair would sit inside
        # the connector exit window cut below
        for s, hole_r, tabs in (
            (+1, WHEEL_HOLE_R, TOP_TABS),
            (-1, IDLER_HOLE_R, [t for t in BOT_TABS if t[0] < -30]),
        ):
            pad = box(length - 36, length, -16, 16, s * tabf, s * inner)
            pad += cyl_z(16, s * tabf, s * inner, x=length)
            p += pad
            p -= cyl_z(hole_r, s * (tabf - 1), s * (outer + 1), x=length)
            if s > 0:  # case top ridge groove along the whole pad (insertion)
                p -= box(length - 37, length + RIDGE_X1 + 1.5,
                         -7.5, 7.5, tabf - 1, TAB_TOP + 1.45)
            else:
                # connector bay groove along the whole pad (insertion) ...
                p -= box(length - 37, length + CONN_X1 + 1.0,
                         -9.7, 9.7, -tabf + 1, -inner - 1.4)
                # ... and a window through the side plate so the plugs can be
                # mated from outside and the wires exit below the link
                p -= box(length - 28.2, length - 9, -10, 10,
                         -inner + 0.1, -outer - 1)
            for tx, ty in tabs:
                hole = cyl_z(M2_CLEAR, s * (TAB_TOP - 2), s * (outer + 1),
                             length + tx, ty)
                cb = cyl_z(2.3, s * (outer - 2.0), s * (outer + 1),
                           length + tx, ty)  # counterbore, head flush
                p -= hole + cb
        if length > 80:  # zip-tie holes to dress the wires along the outside
            for zx in (42, 64):
                for zy in (-5, 5):
                    p -= cyl_z(1.75, -inner + 0.1, -outer - 1, zx, zy)
    else:  # deck
        deck = box(length, length + 2.5, -16, 40, -outer, outer)
        deck -= cyl_x(WHEEL_HOLE_R, length - 1, length + 3.5, y=0, z=0)
        deck -= box(length - 1, length + 3.5, 8.0, 35.3, -7.5, 7.5)  # ridge
        for ny, nz in ((29.0, 10.25), (29.0, -10.25), (8.3, 10.25), (8.3, -10.25)):
            deck -= csk_x(y=ny, z=nz, x_surf=length + 2.5, toward_pos_x=True)
        p += deck
    return p


def base_part():
    """Base plate + box body + top deck clamping servo 1 vertically.

    The plate has a large opening under the servo: the servo is inserted
    from below (wheel up through the deck hole, tabs against the deck
    underside), and its mated plugs hang into the opening.  Wires leave
    through a groove milled in the plate underside toward the -x edge.
    """
    deck_z0 = BASE_H + TAB_TOP + FIT_TAB  # 46.1
    deck_z1 = deck_z0 + CLAMP_T           # 48.6
    p = box(-62.5, 37.5, -40, 40, 0, 8)   # base plate
    p = fillet(p.edges().filter_by(Axis.Z), 8)
    for bx, by in ((-54, -32), (-54, 32), (29, -32), (29, 32)):
        p -= cyl_z(2.25, -1, 9, bx, by)
        p -= cyl_z(4.0, 4.5, 9, bx, by)   # M4 head counterbore
    walls = box(-41.2, 16.2, -18.4, 18.4, 8, deck_z0)
    walls -= box(-37.2, 12.2, -14.4, 14.4, 7, deck_z0 + 1)
    walls -= box(-42.2, -36.2, -8, 8, 14, 30)   # rear hand/cable slot
    deck = box(-41.2, 16.2, -18.4, 18.4, deck_z0, deck_z1)
    deck -= cyl_z(WHEEL_HOLE_R, deck_z0 - 1, deck_z1 + 1)
    deck -= box(RIDGE_X0 - 1.5, RIDGE_X1 + 1.5, -7.5, 7.5,
                deck_z0 - 1, BASE_H + TAB_TOP + 1.45)
    for tx, ty in TOP_TABS:
        deck -= csk_z(tx, ty, deck_z1, up=True)
    p = p + walls + deck
    p -= box(-36.7, 11.7, -13.4, 13.4, -1, 9)   # servo/plug opening
    p -= box(-63.5, -20, -7, 7, -1, 4)          # underside cable groove
    return p


def flange_part():
    """Tool flange bolted to servo 6's wheel."""
    p = horn_disc()
    top0 = WHEEL_FACE + PLATE_T
    p += cyl_z(18, top0, top0 + PLATE_T)
    for i in range(6):
        a = i * 60
        p -= cyl_z(M3_CLEAR, top0 - 1, top0 + PLATE_T + 1,
                   14 * Vector(1, 0, 0).rotate(Axis.Z, a).X,
                   14 * Vector(1, 0, 0).rotate(Axis.Z, a).Y)
    for hx, hy in HORN_HOLES:  # M3 driver access
        p -= cyl_z(ACCESS_R, top0 - 1, top0 + PLATE_T + 1, hx, hy)
    return p


# ------------------------------------------------------------- kinematics --
servo_raw = import_step("STS3215_03a.step")
SHIFT = Location((-12.5, 0, 0))  # put output axis at the frame origin

L1 = Location((0, 0, BASE_H))
T_clamp = Plane(origin=(0, 0, CLAMP_DZ), x_dir=(0, 0, 1), z_dir=(0, 1, 0)).location
T_link = Location((ARM_L, 0, 0))
T_deck = Plane(origin=(DECK_L - TAB_TOP, 0, 0), x_dir=(0, -1, 0), z_dir=(1, 0, 0)).location

BRACKET_NAMES = [
    "b1_shoulder_mount", "b2_upper_arm", "b3_forearm",
    "b4_wrist_mount", "b5_wrist_link", "b6_tool_flange",
]
# transform from joint i's *driven* frame to servo i+1's frame
CHAIN = [T_clamp, T_link, T_deck, T_clamp, T_deck]

_brackets = None


def bracket_parts():
    """Unplaced bracket geometry (built once, cached)."""
    global _brackets
    if _brackets is None:
        _brackets = {
            "base": base_part(),
            "b1_shoulder_mount": clamp_mount(),
            "b2_upper_arm": u_link(ARM_L, NARROW, "pads"),
            "b3_forearm": u_link(DECK_L, WIDE, "deck"),
            "b4_wrist_mount": clamp_mount(),
            "b5_wrist_link": u_link(DECK_L, NARROW, "deck"),
            "b6_tool_flange": flange_part(),
        }
    return _brackets


def build_arm(angles=(0, 0, 0, 0, 0, 0)):
    """Assemble the arm at the given pose.

    angles: J1..J6 in degrees.  Each joint rotates about the output axis
    (local +Z) of its servo; all zero = the arm pointing straight up.
    Returns a Compound whose children are the 6 servos and 7 brackets.
    """
    br = bracket_parts()
    parts = []

    base = br["base"].moved(Location())
    base.label, base.color = "base", Color(0.15, 0.15, 0.15)
    parts.append(base)

    loc = L1  # servo 1 frame
    for i in range(6):
        s = copy.copy(servo_raw).moved(loc * SHIFT)
        s.label = f"servo_{i + 1}_STS3215"
        s.color = Color(0.25, 0.25, 0.28)
        parts.append(s)

        driven = loc * Rotation(0, 0, angles[i])  # joint i+1 motion
        b = br[BRACKET_NAMES[i]].moved(driven)
        b.label = BRACKET_NAMES[i]
        b.color = Color(0.95, 0.55, 0.10)
        parts.append(b)

        if i < 5:
            loc = driven * CHAIN[i]  # next servo rides on this bracket

    asm = Compound(children=parts)
    asm.label = "robot_arm_6dof_sts3215"
    return asm


def check_collisions(asm, tol=0.01, verbose=True):
    """Pairwise interference check; returns list of (label_a, label_b, mm3)."""
    parts = list(asm.children)
    hits = []
    for i in range(len(parts)):
        for j in range(i + 1, len(parts)):
            a, b = parts[i], parts[j]
            ba, bb = a.bounding_box(), b.bounding_box()
            if (ba.min.X > bb.max.X or bb.min.X > ba.max.X or
                    ba.min.Y > bb.max.Y or bb.min.Y > ba.max.Y or
                    ba.min.Z > bb.max.Z or bb.min.Z > ba.max.Z):
                continue
            try:
                inter = a.intersect(b)
                if inter is None:
                    v = 0
                elif hasattr(inter, "volume"):
                    v = inter.volume
                else:  # ShapeList: several disjoint intersection lumps
                    v = sum(s.volume for s in inter)
            except Exception as e:
                if verbose:
                    print(f"  WARN {a.label} x {b.label}: intersect failed ({e})")
                v = 0
            if v > tol:
                hits.append((a.label, b.label, v))
                if verbose:
                    print(f"  OVERLAP {a.label} x {b.label}: {v:.2f} mm^3")
    if verbose and not hits:
        print("  no overlaps")
    return hits


if __name__ == "__main__":
    import argparse
    import os

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--pose", default="0,0,0,0,0,0",
                    help="J1..J6 angles in degrees, comma separated")
    ap.add_argument("--no-check", action="store_true",
                    help="skip the pairwise interference check")
    args = ap.parse_args()
    pose = tuple(float(a) for a in args.pose.split(","))
    assert len(pose) == 6, "--pose needs 6 comma separated angles"

    asm = build_arm(pose)
    os.makedirs("out/parts", exist_ok=True)
    export_step(asm, "robot_arm.step")
    for name, p in bracket_parts().items():
        export_step(p, f"out/parts/{name}.step")
    print(f"exported robot_arm.step (pose {pose}) and out/parts/*.step")

    if not args.no_check:
        print("collision check:")
        check_collisions(asm)
