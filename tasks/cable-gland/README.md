# Fit a cable through a cable gland and tighten it snug (two arms)

Electricians and control-cabinet builders fit a cable gland wherever a cable enters an enclosure: the gland keeps the
box sealed against water and dust (its IP rating) and anchors the cable so that a pull on it doesn't reach the
terminals. A grey polycarbonate box with an M20 cable gland in its near wall stands fixed on the table; the gland's
cap nut is loose and its seal open. A control cable, its end prepared (sheath stripped back, cores taped), lies on
the table in a curve, its far end in a cable clamp. The arms must push the cable through the gland until the sheath,
not just the cores, reaches into the box, then tighten the cap nut **snug**: far enough that the seal grips the
cable, not so far that it crushes the sheath.

The judgment tested is how far to tighten: the nut's turn is scored against marks, under-tightening and
over-tightening lose credit, and crushing the sheath scores nothing. A pull test checks that the cable is anchored.

All parts are our own simplified designs (no manufacturer CAD; see *How the parts are built*).

**Status:** the scripted two-arm demo reaches the full score (1.0, success at step 2,671 of 6,000); four variants
prove the ladder (0.75, 0.5, 0.25, 0). Dream-1 (one trial through `inspect-robots run`, the command below) scores
0.0: see *Dream-1 run*.

## Files

| file | what it is |
| --- | --- |
| `task.py` | the yam_arena task file: scene (`OBJECTS`), `success`, `score`, `distance`; docstring = what is on the table |
| `cg_parts.py` | the custom parts (box, gland with its cap nut and seal, cable): geometry, USD writer, the per-step update (seal contact, crushing, pull test) and the hook that lets yam_arena spawn them |
| `demo.py` | scripted two-arm demo, closed loop on the simulated object poses, and its variants |
| `cg_bench.py` | test bench for the gland alone (the table under *Gland bench*) |
| `preview.py` | renders the starting layout from the station cameras |
| `runs/dream1/` | the Dream-1 run: `eval_log.json.gz` (the inspect-robots log, gzipped); its video is the task's uploaded model-run video |
| `runs/full_video/` | the full demo run: `result.json`, `setup.png` (the starting layout); its video (the three cameras in one) is the task's uploaded demo video |
| `task.json` | submission metadata |

## Run and score a trial

Environment: yam_arena (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, PhysX at 240 Hz). Run at 20 Hz control with object
jitter off (one fixed layout, no seeds needed):

    inspect-robots run --embodiment yam_arena -E task=task.py -E control_hz=20 -E object_jitter=0 -E device=cpu \
        --policy <policy> --scorer task_score --max-steps 6000 \
        --instruction "Fit the cable through the cable gland into the box until its sheath reaches at least 5 mm past the gland's inner end, then tighten the cap nut snug, between the marks, neither short of them nor past them. Then let go of both."

CPU physics (`-E device=cpu`) is what the task was built and tested on (the seal's contacts on the cable are
sub-millimetre; for this one-environment scene CPU PhysX is also faster). The cameras render on the GPU.

`task_score` gives 1.0 when the episode ends in success, otherwise `score()` at the last step. Runtime per trial:
about 30 s of simulator start-up, then about 8 control steps per second on CPU PhysX with the three cameras rendering
(the scripted demo: 2,731 steps in 355 s with 640 x 360 video); a full 6,000-step trial is about 13 minutes.

Scripted demo (drives the same embodiment and task-file scoring directly; CPU physics by default):

    python demo.py --variant full --out demo_runs/full      # result.json, steps.jsonl, one video per camera
    python demo.py --variant all --no-video                  # full, overtight, loose, no_tighten, crush in one process

Gland bench (a few minutes): `python cg_bench.py`, writes `bench_runs/cg_bench.json`.

## Scoring

| score | condition |
| --- | --- |
| 1.0 | **snug**: sheath past (below), the cable holds the pull test, and the nut between the marks (0.4 to under 1 turn past the seal's first contact); both hands off, cable and nut still for 1 s (success, ends the run) |
| 0.75 | **over-tightened**: sheath past, holds, but the nut past the upper mark (1 turn or more past first contact, under 1.3 turns) |
| 0.5 | **holds but short**: sheath past, holds, but the nut short of the lower mark |
| 0.25 | **sheath past**: the cable runs through the gland and its sheath reaches at least 5 mm past the gland's inner end |
| 0 | otherwise, or the sheath **crushed**: the nut ever turned 1.3 turns or more past first contact (even if loosened again) |

- **Sheath past:** the sheath's cut end at least 5 mm past the gland's inner end (the locknut's face inside the box;
  the seal sits further out, so this also puts the sheath past the seal), and the cable runs through the gland (its
  centreline crosses the wall inside the gland's bore: a cable laid over the wall into the open box doesn't count).
  Measured before the pull test (ruler), as in a real check.
- **First contact:** the seal grips the cable all round (every seal pad pushed back by the cable, for 0.25 s): in a
  real trial, the point where the nut becomes hand-tight; mark the nut and the body there.
- **Marks:** turns of the nut since first contact, counted with the marks: 0.4 and 1.0 turn (snug between), 1.3 turns
  (crushed). Turned 1.3 turns or more, the sheath counts as crushed for the rest of the run.
- **Pull test:** run by the task itself once no fingertip has been within 20 mm of the cap nut or of the cable's last
  15 cm before the gland for 1 s, and the seal grips the cable: a pull outward along the gland's axis on the cable
  just outside the cap, ramped to 30 N over 1 s and held 0.5 s. The cable holds if its sheath end moves less than
  2 mm. The result counts until the nut turns 0.02 turn or the sheath moves 1 mm; then the test runs again once the
  hands are off. Real trial: a spring scale on the cable, 30 N.
- **Still:** the sheath end moving under 10 mm/s (measured over each control step) and the nut turning under
  0.5 rad/s.
- Measurable with a ruler, a marker, a spring scale and a photo.

No unintended strategy scores: tightening without the cable through, or with only the taped cores in the seal,
scores 0 (sheath not past); a cable dropped over the wall into the box doesn't run through the gland; pushing the
cable in without tightening caps at 0.25 (the seal doesn't grip, so it can't hold); turning the nut far past the marks
scores 0 for good; the box and the gland's body are fixed. Holding the cable or the nut at the end leaves the pull
test unrun (no more than 0.25).

## Setup

All positions from the arm bases' midline (x forward, y to the left); the table top is the reference height.

- **Enclosure** (fixed): grey polycarbonate box, 120 mm deep (x) x 160 mm wide (y) x 60 mm tall, 3 mm walls, lid
  off; its near wall's outer face at x 0.27 m, centred on the midline.
- **Cable gland** (fitted in the middle of the near wall, its axis along x, 35 mm above the table): M20 x 1.5,
  black-grey polyamide. Body with a 22 mm hex flange outside the wall and a locknut (24 mm across flats) inside, bore
  13.2 mm; seal insert of six rubber pads (open bore 13 mm, 12 mm long, a 45 degree lead-in); domed cap nut, 24 mm
  across flats, 14 mm hex and a 6 mm dome with a 13 mm mouth. The cap nut starts loose (seal open). About 2 turns
  clockwise (as seen from the arms) bring the seal onto an 8 mm cable.
- **Cable** (loose but for its clamped end): round control cable, 3 cores x 1.5 mm2, grey PVC sheath 8 mm across,
  ~100 g/m; 420 mm of sheath from the clamp, then the prepared end: sheath stripped back 75 mm, the three cores
  (brown, blue, green-yellow, ~3 mm each) taped together. It lies on the table in an arc from the clamp: forward-left,
  then turning right, its sheath's cut end at x 0.16 m, y 0.08 m, the cores pointing toward the box (their tip about
  4 cm in front of the wall, 4 cm left of the gland's axis).
- **Cable clamp** (fixed): black plastic block, 24 x 22 x 18 mm, on the table at x 0.10 m, y -0.24 m, holding the
  cable's far end, the cable leaving it pointing forward-left (30 degrees from x). It stands in for the rest of the
  cable's run.

## How the parts are built

yam_arena's task files only offer single primitives and library assets, and yam_arena has no extension point for
jointed parts. `cg_parts.py` is the task's own minimal mechanism; nothing in yam_arena is modified. A `Part`
subclasses yam_arena's `Box`; `task.py` calls `cg_parts.install()`, which wraps `yam_arena.scene.task_object`,
`object_states` and `make_env` when that module is imported, and spawns the parts from USD files it writes to a
temporary folder. The `make_env` wrapper also resets the gland and the cable with each episode (yam_arena resets the
robot and free objects only).

**Geometry and licensing:** every part's geometry was created by us for this task and is written into `cg_parts.py`
as coordinates; no manufacturer CAD, meshes or other third-party assets are used. The gland follows the metric
thread M20 x 1.5 (ISO 261) and the general layout of a cable gland for electrical installations (body, seal insert,
cap nut); all its dimensions are our own choices. The cable's size is that of a common 3 x 1.5 mm2 control cable;
its stiffness is our estimate. The robot, table and cameras are yam_arena's own, unchanged.

**The gland** is one world-fixed articulation:
- **Cap nut:** a prismatic joint (the nut's travel along the axis) coupled to the nut's turning by a PhysX mimic
  joint at 1.5 mm per turn; constant thread friction 0.08 Nm (it holds the nut where the hand leaves it).
- **Seal:** six pads round the bore. Each is driven inward by the nut's travel (a mimic joint: 0.6 mm radially per mm
  of travel, the cap's cone) and sits on a radial spring (80 N/mm, the rubber's give). The pads squeeze the cable by
  contact, so the grip and the pull resistance come out of the physics. The pads sit on a ring that the nut drags
  round through a slipping clutch (a drive capped at 0.03 Nm), as a seal turns with its cap until the cable resists.

**The cable** is an articulation with its clamp end fixed: 20 mm capsule links (r 4 mm) joined by two bending joints
and a twisting joint each, with springs for its bending and torsional stiffness (EI 0.006 N m2, GJ 0.004 N m2), then
three thinner, softer core links (r 3.2 mm). Its laid curve is built into the USD (the springs rest in it), and the
bending springs' rest angles creep toward the current shape (the set a PVC cable takes; only the part of a bend beyond
6 degrees creeps, time constant 1.5 s).

**The per-step update** (called from the patched `object_states` once per control step, before scoring) follows the
seal's first all-round contact, counts the nut's turns since, latches "crushed", and runs the pull test.

### Gland bench

`cg_bench.py`: four glands side by side, each with a short cable (160 mm of sheath) already through it (sheath end 11
mm past the gland's inner end), the arms parked. A torque-limited "virtual hand" turns each cap nut (a quarter turn
per second, at most 0.5 Nm) to a set number of turns past first contact and lets go; the task's own pull test follows.

| case | nut turned past first contact | pull test (30 N) |
| --- | --- | --- |
| a | 0.20 turn | held (moved 0.001 mm) |
| b | 0.30 turn | held (0.000 mm) |
| c | 0.65 turn | held (0.001 mm) |
| d | 1.12 turns | held (0.001 mm) |

First contact came 2.15-2.16 turns from the start in all four; each nut stayed where it was left. In the scene, the
demo variants confirm the bands with the full-length cable (next section). Results in `bench_runs/cg_bench.json`.

## The demo

Left arm takes the cable 15 mm behind the sheath's cut end from above, lifts it and carries it onto the gland's axis
30 mm in front of the cap (closed loop on the measured cores' tip: the soft wrist and the cable's droop leave it a few
mm off), pushes the taped cores in through the cap's mouth, lets go, takes the cable again further back and pushes
until the sheath's end is 15-18 mm past the gland's inner end. It then takes the cable 100 mm in front of the cap,
its hand tilted 40 degrees toward its own side about the cable (to leave room for the other gripper), and holds it.

Right arm grips the cap nut across two flats, its hand leaned back 25 degrees (clear of the box's wall) and turned to
the flats' angle, and turns it clockwise in wrist strokes about the gland's axis (rolling the hand from the flats'
angle, between 0 and 60 degrees, to 70 degrees; re-gripping between strokes), until the nut has turned the target past
the seal's first contact. Both hands move away; after 1 s hands off the task pulls the cable.

Measured (CPU PhysX, 20 Hz control, jitter 0): **success at step 2,671** of 6,000 (score 1.0): the sheath through at
step 916, 17 strokes, first contact 2.08 turns from the start, stopped 0.66 turn past it; pull test 0.003 mm. The
video runs 3 s past success.

Variants proving the ladder (all five in one process, `--variant all`; each also run on its own, with the same scores):

| variant | what the arms do | score | steps |
| --- | --- | --- | --- |
| `full` | as above, 0.66 turn past first contact | **1.0** (success) | 2,671 |
| `overtight` | the same, but 1.14 turns past first contact | **0.75** | 2,993 |
| `loose` | the same, but 0.31 turn past first contact (holds: 0.008 mm) | **0.5** | 2,522 |
| `no_tighten` | the cable pushed through and let go, the nut not touched | **0.25** | 971 |
| `crush` | the same as `full`, but 1.47 turns past first contact | **0** (crushed) | 3,145 |

## Dream-1 run

One trial with the command under *Run and score a trial* (`--policy dream`, CPU PhysX, cameras 320 x 176,
6,000 steps, 499 s): **task_score 0.0** (hit the step limit). Both arms reach toward the box and the cable early on;
the right hand then hovers about the cable clamp and the cable beside it, the left arm moves back toward its base.
Neither hand picks up the cable or touches the cap nut; the cable stays where it started. Log in `runs/dream1/`; the
video is the task's uploaded model-run video.

## Fidelity: what is simplified

- The thread is a joint, not thread contact: 1.5 mm per turn with a constant friction of 0.08 Nm, far below the
  torque a real cap nut needs; the nut can't cross-thread or come off.
- "Crushed" is judged by the nut's turns past first contact (1.3 turns), not by modelling damage to the sheath; the
  seal's squeeze is a radial spring per pad, and the measured pad deflection stays small (~0.1 mm). The hold the seal
  gives the cable comes from the contact friction of the pads (rubber 0.8) on the sheath.
- The cable is a chain of rigid 20 mm links with springs; its "set" (staying where it is bent) is a creep of the
  springs' rest angles. Its stiffness values are our estimates for a small PVC control cable.
- The seal turns with the cap nut through a 0.03 Nm clutch.
- The pull test is applied by the task (an external force on the cable link just outside the cap), not by a robot.
- Friction: rubber seal 0.8, PVC sheath 0.5, polyamide/polycarbonate 0.4, the robot's fingers PhysX default 0.5.
  Contact offset 0.5 mm on the parts.

## Known weak points

- The scripted demo needs closed-loop corrections (the YAM's soft wrist) and is tuned to this layout: the two
  grippers cannot work within about 10 cm of each other at the gland, so one arm holds the cable 10 cm back while the
  other turns the nut. Runs from the start position repeat exactly.
- Holding the cable while the nut is turned is good practice and the demo does it, but it isn't scored: we tried a
  rule for the cable twisting with the seal, but in this layout a cable lying on the table resists turning about as
  firmly as a hand, so it couldn't tell holding from not holding.
- A cable that fails the pull test is pulled back out by the test; the score then uses the sheath's position from
  before the test.
- `cg_parts.install()` wraps three functions of `yam_arena.scene`; a yam_arena update that changes them stops the task
  with an error. It must run before `yam_arena.scene` is first imported (true for `-E task=task.py`).
- The scripted demo runs through `demo.py` (the same `YamArenaEmbodiment` and task-file scoring as `inspect-robots
  run`), not the CLI.
