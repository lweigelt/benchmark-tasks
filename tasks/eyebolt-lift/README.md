# Line up a lifting eyebolt with the chain's pull and hook on (two arms)

A shouldered lifting eyebolt may only be loaded **in the plane of its eye**. Pulled from the side, across the eye, its
capacity drops sharply (the rule behind DIN 580 and ASME B18.15). Screwed down to its shoulder, an eyebolt's eye
usually ends up pointing somewhere else than the sling. The correct fix is to keep **tightening** past the shoulder
until the eye lines up, never to back it off, which would leave the shoulder loose. Only then does the hook go on,
with its safety latch closed over the eye.

On the table: a machine part with a tapped M16 hole, an M16 eyebolt standing in a tray, and a lifting chain hanging
from an overhead bracket to the right with a latched hook on its end. The arms must:
1. screw the eyebolt down to its shoulder;
2. tighten it further until its eye is in line with the chain (it stops 75 degrees short at the shoulder);
3. hook the chain's hook into the eye, so the spring latch snaps closed over it, and let go.

What the task tests, beyond screwing a part home:
- **alignment under a one-way constraint:** the last 75 degrees are a precise rotation against the shoulder's rising
  clamp. Overshooting or backing off loses the seat.
- **hooking a latched hook:** the hook's tip goes through the eye and the eye's section pushes the latch open, then
  drops into the bowl.
- **safe sequencing:** a hook in an eyebolt that isn't seated and in line scores 0.

All parts are our own simplified designs (no manufacturer CAD; see *How the parts are built*).

**Status:** the scripted two-arm demo reaches the full score (1.0, success at step 4,633 of 7,000), and the variants
score as the ladder says (0.75, 0.5, 0.25). Dream-1 (one trial through `inspect-robots run`, the command below)
scores 0.0: see *Dream-1 run*.

## Files

| file | what it is |
| --- | --- |
| `task.py` | the yam_arena task file: scene (`OBJECTS`), `success`, `score`, `distance`; docstring = what is on the table |
| `el_parts.py` | the custom parts (housing, eyebolt with its thread mechanism, tray, overhead bracket, chain with latched hook): geometry, USD writer, and the hook that lets yam_arena spawn them |
| `demo.py` | scripted two-arm demo, closed loop on the simulated object poses, and its variants |
| `hook_plan.py`, `hook_path.json` | offline planner for the hook's way into the eye (the demo follows `hook_path.json`) |
| `el_bench_thread.py` | test bench for the thread mechanism alone (*Benches*) |
| `el_bench_hook.py` | test bench for the hook and latch alone: a "virtual hand" follows the planned way in (*Benches*) |
| `runs/dream1/` | the Dream-1 run: `eval_log.json.gz` (the inspect-robots log, gzipped); its video is the task's uploaded model-run video |
| `runs/full_video/` | the full demo run: `result.json`, `setup.png` (the starting layout); its video (the three cameras in one) is the task's uploaded demo video |
| `task.json` | submission metadata |

## Run and score a trial

Environment: yam_arena (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, PhysX at 240 Hz). Run at 20 Hz control with object
jitter off (one fixed layout, no seeds needed):

    inspect-robots run --embodiment yam_arena -E task=task.py -E control_hz=20 -E object_jitter=0 -E device=cpu \
        --policy <policy> --scorer task_score --max-steps 7000 \
        --instruction "Fit the lifting eyebolt into the part for a lift: screw it into the tapped hole down to its shoulder, keep tightening until its eye is in line with the chain (never back it off), then hook the chain's hook into the eye and let go."

The task was built and tested on CPU physics (`-E device=cpu`): the thread and latch contacts are small, and for this
one-environment scene CPU PhysX is also faster. The cameras render on the GPU.

`task_score` gives 1.0 when the episode ends in success, otherwise `score()` at the last step. Runtime per trial:
about 30 s of simulator start-up, then about 10 control steps per second on CPU PhysX with the three cameras
rendering. The scripted demo took 4,693 steps in 464 s with 640 x 360 video; the Dream-1 trial took 7,000 steps in
604 s, including model inference. A full 7,000-step trial is about 10-12 minutes.

Scripted demo (drives the same embodiment and task-file scoring directly; CPU physics by default):

    python demo.py --variant full --out demo_runs/full      # result.json, steps.jsonl, one video per camera
    python demo.py --variant hold --no-video                # likewise hold, no_hook, no_align

Benches (a few minutes each): `python el_bench_thread.py --device cpu` and
`LATCH_HOLD=0 python el_bench_hook.py --device cpu`. They write JSON to `bench_runs/`.

## Scoring

| score | condition (each includes the ones below it) |
| --- | --- |
| 1.0 | **let go**: hooked (below), and no robot finger within 15 mm of the hook (**success**: also the hook still for 1 s; ends the episode) |
| 0.75 | **hooked**: seated and in line (below), and the hook through the eye with its latch closed |
| 0.5 | **in line**: seated (below), and the eye's plane within 15 degrees of the chain's direction, seen from above |
| 0.25 | **seated**: the eyebolt screwed in down to its shoulder: the collar's underside within 0.5 mm of the part's face, pressing on it |
| 0 | otherwise; and **always 0 if the hook is in the eye of an eyebolt that is not seated or not in line** (the load would come on it loose or from the side) |

- **Seated:** in a real trial, a 0.5 mm feeler gauge doesn't go under the collar anywhere. In the simulation: the
  eyebolt is in its thread, its collar is within 0.5 mm of the face, and the collar has pressed on the face with
  2 N or more for 0.5 s in a row (its clamp spring; see Fidelity).
- **In line:** the angle between the eye's plane and the chain's direction (from the hole to the bracket's anchor,
  seen from above), at most 15 degrees. Real trial: a photo from above, or a protractor.
- **Hooked:** the eye passes through the hook's bowl, below its closed latch (latch within 10 degrees of closed), in
  the hook's plane. Real trial: a photo; the latch is shut.
- **Let go:** no part of either gripper's fingers within 15 mm of the hook. Real trial: a ruler, or a photo.
- **Still:** under 10 mm/s and 0.5 rad/s.
- Everything is measurable with a feeler gauge, a protractor, a ruler and a photo.

No unintended strategy scores:
- **Backing off to line up the eye loses the seat.** At the shoulder the eye is 75 degrees short, so backing off
  would need 105 degrees, which lifts the collar 0.58 mm and takes its pressure off.
- **Standing the eyebolt in the hole and turning it to point along the chain** doesn't seat it: without the thread
  there is no collar pressure.
- **Hooking the eyebolt in the tray, in the hole, or half screwed in** scores 0, as does hooking it out of line.
- **Pulling the chain to the eyebolt doesn't help:** the bracket and the part are fixed, and the chain only reaches
  from where it hangs.

## Setup

All positions from the arm bases' midline (x forward, y to the left); the table top is the reference height.

- **Machine part** (fixed): a gearbox housing simplified to a painted cast-iron block, 160 x 120 x 40 mm, centred
  0.30 m forward and 0.02 m right. One tapped M16 hole in the middle of its top: 14 mm core, 90 degree countersink to
  17 mm, the top wall 32 mm thick.
- **Tray** (loose, 0.35 kg): steel block 60 x 60 x 32 mm with a 17 mm hole, centred 0.22 m forward and 0.17 m left.
- **Eyebolt** (loose, 0.28 kg): M16 lifting eyebolt, forged steel, DIN 580 proportions:
  - eye 63 mm outside, 35 mm inside, 14 mm section;
  - collar 35 mm across and 8 mm high;
  - thread 27 mm long, 2 mm pitch.

  It stands in the tray thread down, its collar on the tray, the eye's plane pointing left-right.
- **Overhead bracket** (fixed): black steel post 40 x 40 mm, 0.52 m forward and 0.16 m right, on a 120 x 120 x 10 mm
  foot. A 30 x 20 mm arm runs back toward the arms to the chain's anchor point, 0.30 m forward, 0.16 m right and
  0.55 m above the table: 14 cm to the right of the hole.
- **Chain and hook:**
  - chain: 16 links of 30 mm (4 mm round steel, 10 g each), hanging from the anchor;
  - hook: an 8 mm-section steel hook, 0.12 kg, with a 22 mm throat, a bowl 38 mm across and a spring safety latch (5 g)
    that opens inward;
  - the hook hangs just above the table.

The eyebolt catches its thread when it stands upright in the hole and is turned clockwise. From there about 12.5 turns
bring its collar down onto the part's face. There its eye stops 75 degrees short of the chain's direction (counter-
clockwise of it, seen from above), and tightening it further turns the eye in line.

## How the parts are built

yam_arena's task files only offer single primitives and library assets, and yam_arena has no extension point for
jointed parts. `el_parts.py` is the task's own minimal mechanism; nothing in yam_arena is modified. A `Part`
subclasses yam_arena's `Box`. `task.py` calls `el_parts.install()`, which wraps `yam_arena.scene.task_object` and
`object_states` when that module is imported, and spawns the parts from USD files it writes to a temporary folder.

**Geometry and licensing:** every part's geometry was created by us for this task and is written into `el_parts.py`
as coordinates. No manufacturer CAD, meshes or other third-party assets are used.
- The eyebolt follows DIN 580's published proportions for M16.
- The housing, tray, bracket, chain and hook are our own designs.
- The robot, table and cameras are yam_arena's own, unchanged. The "let go" check reads the robot fingers' collision
  meshes from yam_arena's robot description.

**The thread.** A loose eyebolt can't carry a permanent thread joint, so it exists twice:
- the **loose eyebolt**, an ordinary rigid body. Its collar bears on 12 small balls on a ring, and its thread's tip is
  chamfered.
- its **threaded twin**, an articulation fixed to the world and parked out of sight:
  - an anchor;
  - a thread slider coupled to the eyebolt's turning by a PhysX mimic joint (2 mm per turn);
  - a stiff clamp spring under the collar (100 N/mm, 0.8 mm travel: about 144 degrees past contact);
  - the eyebolt turning against a constant 0.05 Nm thread friction.

  The thread's start is fixed by the hole, as in a real tapped hole, so the eye always stops at the same angle at the
  shoulder.

When the loose eyebolt stands in the hole and is turned clockwise 20 degrees, the twin takes its place at the same
position and angle: it has caught its thread. "Standing in the hole" means its tip within 1.2 mm of the hole's axis,
down in the countersink, and upright within 3 degrees. Turned back out a quarter turn past where it caught, it is
loose again. The swap runs once per control step, before scoring.

**The chain and hook.** One articulation hangs from the anchor:
- 16 rigid links, each joined to the next by two revolute joints (a universal joint);
- the hook;
- the latch, on a hinge with a weak spring.

The latch is hinged high on the hook's shank and slants down to the tip. Pushed by the eye's 14 mm section, it opens
up to 54 degrees, wide enough to let the section pass. It closes above the section once the section rests in the
bowl.

## Benches

**Thread** (`el_bench_thread.py`): three eyebolts standing loose in three holes (each hole's thread laid out the same
way), plus one in the tray. A "virtual hand" turns an eyebolt with a torque-limited speed controller (1 turn per
second, at most 0.1 Nm), holding it upright with a weak torque and pressing lightly until it catches, then 10 N.

| case | result |
| --- | --- |
| eyebolt standing in a hole, turned (3 holes; two starting angles: 0 and 50 degrees) | catches after 0.76 s, all 3 |
| turned down to the shoulder | contact after 12.7-12.8 turns; the eye then 57-64 degrees short of the pull when contact is confirmed (it stops 75 degrees short at first touch; confirming takes 0.5 s of turning) |
| tightened on until in line | in line 0.8 and 1.7 degrees off after 0.11-0.13 turn past contact, clamp 24-25 N; let go, it stays (2.6 degrees off, 26 N) |
| seated eyebolt turned back out | loose again a quarter turn past where it caught; lifts out freely |
| eyebolt standing in the tray, untouched | rests still (tilt 0.001 degree) |

**Hook** (`el_bench_hook.py`): the eyebolt seated in the part, in line with the chain. A "virtual hand" (a PD force
and torque on the hook) takes the hanging hook along the planned way in (`hook_path.json`), latch not held, then holds
it 1 s and lets go for 3 s.

| case | result |
| --- | --- |
| following the way in | within 7.3 mm of the path; the eye's section pushes the latch open and drops into the bowl |
| holding, then letting go | hooked with the latch closed (within 2 degrees); stays hooked the whole 3 s, chain pulling |

Results in `bench_runs/`.

## The demo

Both arms work at once (`demo.py`, closed loop on the simulated object poses; the YAM's wrist is soft and lags its
command by a few millimetres, so the demo corrects on what it measures).

**Left arm, the eyebolt.**
1. Takes the eyebolt from the tray by the top of its eye (fingers closing across the eye).
2. Carries it upright over the hole and centres its tip over the hole: correct, settle, measure. It ends within
   0.4 mm of the hole's axis.
3. Sets it into the countersink and turns the hand 30 degrees clockwise about its axis, keeping it upright: the
   thread catches.
4. Screws it down in wrist strokes of up to half a turn: grip the eye, turn about the eyebolt's axis following it
   down, let go, lift, turn back in the air. Joint 6 has only 240 degrees, so each stroke is planned ahead with
   chained IK.
5. At the shoulder it regrips and turns just as far as the eye needs to come in line, then moves away.

**Right arm, the hook** (it starts once the thread has caught, so the two hands don't get in each other's way).
1. Slides onto the hanging hook in the hook's plane, so the chain passes between its fingers, and grips the shank
   (centred on the measured hook).
2. Lifts it and holds it clear of the left hand.
3. Once the eye is in line, carries the hook beside the eye and follows a planned way in (`hook_plan.py`, an offline
   search in the plane across the eye with 3 mm clearance). The hook is kept at the 20 degree tilt it hangs at in
   the grip.
4. The tip passes through the eye. The eye's section pushes the latch open and drops into the bowl, and the latch
   snaps shut.
5. The hand lets go and moves away.

Measured (CPU PhysX, 20 Hz control, jitter 0): **success at step 4,633** of 7,000 (score 1.0).
- **Thread:** caught at step 412; 30 strokes, 13.3 turns.
- **Shoulder:** contact at step 3,957; in line at step 4,159, 2.4 degrees off the chain.
- **Hook:** gripped at step 558 (first try); followed the planned way within 1.8 mm; hooked at step 4,583.
- **After letting go:** the hook stays hooked, latch closed.

The video runs 3 s past success.

Variants proving the ladder (CPU PhysX, each run from a fresh start; `full` at success, the others until the arms are
done):

| variant | what the arms do | score | steps |
| --- | --- | --- | --- |
| `full` | as above | **1.0** (success) | 4,633 |
| `hold` | as above, but the right hand keeps holding the hook | **0.75** | 4,645 |
| `no_hook` | eyebolt seated and in line (2.4 degrees); the right arm stays put | **0.5** | 4,202 |
| `no_align` | eyebolt seated, stopped there (eye 28.5 degrees off the chain); the right arm stays put | **0.25** | 4,026 |

## Dream-1 run

One trial with the command under *Run and score a trial* (`--policy dream`, CPU PhysX, cameras 320 x 176,
7,000 steps, 604 s): **task_score 0.0** (hit the step limit). The left gripper hovers over the eyebolt in the tray
again and again but never closes on it, and the right arm wanders over the table. The eyebolt stays in the tray, and
the chain and hook keep hanging where they started. Log in `runs/dream1/`; the video is the task's uploaded model-run
video.

## Fidelity: what is simplified

- **The thread** is the twin swap above: the eyebolt catches at a fixed rule (upright within 3 degrees, tip in the
  countersink, 20 degrees clockwise) rather than by thread contact. Cross-threading is not modelled: a tilted
  eyebolt simply doesn't catch.
- **Seated is judged by the collar's gap and pressure, not by torque.** The clamp spring gives tens of newtons, far
  below a real tightened M16 eyebolt.
- **The chain's links don't collide** with anything; only the hook and latch do. A real chain gives way when a hand
  reaches past it to the hook, while ours (rigid rods on universal joints) swung the hook away. The chain still
  hangs, swings and pulls on the hook.
- **The hook's section is 8 mm** (a real hook for this chain is thicker), so it fits through the eye with room
  for the soft robot wrist. The hook is our own shape, and its latch spring is weak (0.01 Nm/rad).
- **Friction:** steel 0.3, thread start 0.2, painted housing 0.35. The table and robot fingers use yam_arena's
  defaults. Contact offset 0.5 mm on the parts, rest offset 0.

## Known weak points

- **The scripted demo is tuned to this one layout.** The way in is planned offline for the eye's final position.
  Runs from the start position repeat exactly; each variant was run from a fresh start.
- **The hook can only be gripped from above with the chain passing between the fingers** because the chain links
  don't collide (see Fidelity). With a colliding chain, the gripper's base pushed the chain and swung the hook away.
- **A policy that pulls hard on the hooked chain can't move the eyebolt.** The thread twin only turns about its
  axis, so the eye's alignment survives side loads that would, in reality, bend a misloaded eyebolt. The scoring
  doesn't need that: the alignment is checked on the eye, not on the load.
- **`el_parts.install()` wraps two functions of `yam_arena.scene`.** A yam_arena update that changes them stops the
  task with an error. It must run before `yam_arena.scene` is first imported (true for `-E task=task.py`).
- **The scripted demo runs through `demo.py`** (the same `YamArenaEmbodiment` and task-file scoring as
  `inspect-robots run`), not the CLI.
