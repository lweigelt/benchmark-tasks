"""On the station table, in front of the arms: a machine part (a gearbox housing, simplified to a painted cast-iron
block, 160 x 120 x 40 mm, fixed) with a tapped M16 hole in the middle of its top. Beside it, to the left: a steel
tray with an M16 lifting eyebolt (forged steel, DIN 580 proportions: eye 63 mm outside, 35 inside, 14 mm section,
35 mm collar) standing in it, thread down. To its right: an overhead bracket with a lifting chain hanging from it
(16 links of 30 mm, a hook with a sprung safety latch on its end, hanging just above the table). The chain's pull on
the eyebolt runs from its eye up to the bracket, 14 cm to the right of the hole.

The eyebolt catches its thread when it stands upright in the hole and is turned clockwise; about 12.5 turns bring its
collar down onto the part (the shoulder); there its eye stops 75 degrees short of the chain's direction, and turning
it further (tightening, never backing off) brings it in line: a shouldered eyebolt may only be loaded in the plane of
its eye (loaded across it, its capacity drops sharply).

Task: fit the lifting eyebolt into the part before a lift: screw it down to its shoulder, tighten it until its eye is
in line with the chain, then hook the chain's hook into the eye and let go.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import el_parts as el  # noqa: E402

from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import quat_xyzw_to_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

MM = 1e-3
HOUSING_ORIGIN = (0.30, -0.02, TOP + el.HOUSING_SIZE[2])  # its top face centre; the hole there
HOLE = (HOUSING_ORIGIN[0], HOUSING_ORIGIN[1], HOUSING_ORIGIN[2])
ANCHOR = (HOLE[0], HOLE[1] - 0.14, TOP + 0.55)  # the chain's anchor under the bracket's arm, to the right of the hole
# (behind it, at x 0.42, neither arm reached the hanging hook pointing down: reach scan)
el.GANTRY_H = ANCHOR[2] - TOP
PULL_YAW = math.atan2(ANCHOR[1] - HOLE[1], ANCHOR[0] - HOLE[0])  # the pull's direction seen from above
SHORT = math.radians(75.0)  # the eye stops this far short of the pull at shoulder contact (counter-clockwise of it)
el.set_hole_phases([PULL_YAW + SHORT])
el.install()

TRAY_ORIGIN = (0.22, 0.17, TOP + el.TRAY_SIZE[2] + 0.1 * MM)  # its top face centre
EYEBOLT_START = (TRAY_ORIGIN[0], TRAY_ORIGIN[1], TRAY_ORIGIN[2] - el.THREAD_LEN + 0.3 * MM)

OBJECTS = [
    el.part("housing", "housing", HOUSING_ORIGIN, fixed=True),
    el.part("gantry", "gantry", ANCHOR, fixed=True),
    el.part("tray", "tray", TRAY_ORIGIN),
    *el.eyebolt_parts("eyebolt", EYEBOLT_START, park_y=-0.3, yaw=math.radians(90.0)),
    el.part("sling", "sling", ANCHOR),
]

# -- scoring (checkable by hand and eye: turns marked on the eye, a feeler gauge under the collar, a protractor or a
# photo from above for the eye's direction, a photo of the hook) -------------------------------------------------------
SEATED_GAP = 0.5 * MM  # seated: the collar's underside within this of the part's face (and pressing on it)
ALIGN_DEG = 15.0  # in line: the eye's plane within this of the chain's direction (seen from above)
RELEASE_DIST = 15 * MM  # released: no fingertip within this of the hook


def bolt_tip(st):
    rot = quat_xyzw_to_matrix(np.asarray(st.quat_xyzw, dtype=float))
    return np.asarray(st.pos, dtype=float) - rot @ np.array(el.local_center("eyebolt")), rot


def eye_off_pull_deg(objects) -> float:
    """The eye's plane's angle from the chain's direction seen from above, in [-90, 90) degrees."""
    _, rot = bolt_tip(objects["eyebolt"])
    yaw = math.atan2(rot[1, 0], rot[0, 0])
    return math.degrees((yaw - PULL_YAW + math.pi / 2) % math.pi - math.pi / 2)


def seated(objects) -> bool:
    st = objects["eyebolt"]
    return bool(st.engaged and st.touching and st.gap is not None and st.gap < SEATED_GAP)


def aligned(objects) -> bool:
    return seated(objects) and abs(eye_off_pull_deg(objects)) <= ALIGN_DEG


def hooked(objects) -> bool:
    """The hook through the eye: the eye's top section inside the hook's bowl, below its closed latch."""
    return el.hooked(objects["eyebolt"], objects["sling"])


def released(objects) -> bool:
    return bool(getattr(objects["sling"], "hand_dist", math.inf) > RELEASE_DIST)


def score(objects) -> float:
    """0.25 seated; 0.5 seated with the eye in line with the chain; 0.75 also hooked (latch closed over the eye); 1.0
    also let go (success, held 1 s). A hook in the eye of an eyebolt that isn't seated or isn't in line (the load
    would come on it unseated or from the side): 0."""
    if hooked(objects) and not aligned(objects):
        return 0.0
    if aligned(objects):
        if hooked(objects):
            return 1.0 if released(objects) else 0.75
        return 0.5
    return 0.25 if seated(objects) else 0.0


def success(objects) -> bool:
    """Seated, in line, hooked, let go, and the hook still (held 1 s)."""
    return score(objects) >= 1.0 - 1e-9 and objects["sling"].resting()


def distance(objects) -> float:
    """Metres from the eyebolt's tip to the hole (a progress measure, not the score)."""
    tip, _ = bolt_tip(objects["eyebolt"])
    return float(np.linalg.norm(tip - (np.array(HOLE) - (0, 0, el.SEAT_DEPTH))))
