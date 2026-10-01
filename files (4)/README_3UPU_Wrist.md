# 3-UPU Spherical Wrist — Design Package

## 1. Architecture & why it's *pure rotation*

A plain 3-UPU leg set (3 legs, each Universal–Prismatic–Universal) is only
constrained to pure spherical (rotation-only) motion under very specific
joint-axis alignment conditions that are hard to manufacture with COTS
universal joints. The robust, commonly-used practical solution — used here —
adds **one passive central spherical joint** that pins the platform's
geometric center to a single fixed point **O** in space:

- The central joint removes all 3 translational DOF of the platform (its
  center can never move away from O).
- The 3 active UPU legs, arranged symmetrically at 120° around the central
  joint, each contribute one actuated length. Three actuated lengths exactly
  match the 3 remaining rotational DOF (roll / pitch / yaw about O) — the
  mechanism is fully determined, not redundant or under-constrained.
- Each U-joint (2 passive rotational DOF, at the base and at the platform end
  of every leg) simply lets the leg re-orient itself as required; only the
  ball-screw prismatic joint in the middle is driven.

This is the same principle used in RCM (remote-center-of-motion) wrists and
several patented parallel wrists: **3 active UPU legs + 1 passive central
S-joint → 3-DOF spherical (pure-rotation) motion about a fixed point.**

## 2. Key parameters (mm, deg)

| Parameter | Symbol | Value |
|---|---|---|
| Base U-joint circle radius | R_b | 65 |
| Platform U-joint circle radius | R_p | 38 |
| Height of fixed rotation center O above base plate | H_c | 70 |
| Platform anchor offset above O (local) | dz_p | 8 |
| Base plate radius / thickness | – | 85 / 8 |
| Platform plate radius / thickness | – | 54 / 6 |
| Nominal (home) leg length | L₀ | 71.3 |
| Leg length range (±15° single-axis tilt) | ΔL | ≈ 62.9 – 81.4 (≈ ±10 mm about L₀) |

## 3. Actuation — ball screw + stepper

- **Screw:** SFU1204 miniature ball screw, 12 mm nominal diameter, 4 mm lead.
  (Swap to SFU1605 — 16 mm dia, 5 mm lead — if higher axial stiffness/load
  capacity is needed; it is a drop-in envelope change in the model.)
- **Motor:** NEMA17 stepper, 1.8°/step (200 full steps/rev), driven with a
  microstepping driver (e.g., 16 µsteps/step → 3200 µsteps/rev).
- **Linear resolution:** 4 mm / 3200 µsteps ≈ **1.25 µm/µstep** — far finer
  than needed for a wrist joint, so angular resolution is limited by
  mechanical stiffness/backlash, not the actuator.
- **Recommended screw travel:** ≥ 40–50 mm usable stroke to give margin
  beyond the ±10 mm computed swing, allowing a larger commanded workspace
  (e.g. ±25–30° per axis) and assembly/calibration slack.
- Coupling: motor shaft → ball screw via a flexible beam or Oldham coupling
  (not modeled in detail — call out as a COTS part in the BOM).

## 4. Inverse kinematics (used to drive the CAD frames)

For a commanded platform rotation matrix **R** about the fixed center **O**:

```
P_i(R) = O + R · p_i_local        (platform anchor i, world frame)
L_i(R) = | P_i(R) − B_i |         (required leg length, i = 1..3)
```

where `B_i` are the fixed base anchor points and `p_i_local` are the
platform anchor points expressed in the platform's own (undeformed) frame.
This is exactly what `wrist_model.py` evaluates to generate each motion
frame — i.e., the STEP files aren't just re-oriented by hand, the leg
geometry (screw length + nut position) is recomputed from real IK.

Computed frames (see `leg_lengths.json`):

| Frame | Rx / Ry / Rz (deg) | L1, L2, L3 (mm) |
|---|---|---|
| Home | 0 / 0 / 0 | 71.31, 71.31, 71.31 |
| Tilt X | 15 / 0 / 0 | 81.44, 66.26, 66.26 |
| Tilt Y | 0 / 15 / 0 | 71.09, 80.01, 62.92 |
| Combined | 10 / 10 / 6 | 77.83, 73.93, 62.68 |

## 5. Deliverables in this package

| File | Description |
|---|---|
| `wrist_home.step` | Full assembly, neutral (0°,0°,0°) pose |
| `wrist_tiltX15.step` | Assembly posed at 15° tilt about X |
| `wrist_tiltY15.step` | Assembly posed at 15° tilt about Y |
| `wrist_combo.step` | Assembly posed at combined 10°/10°/6° |
| `wrist_motion_study.gif` | Animated sequence sweeping through the frames above — the kinematic "motion study" |
| `wrist_*.png` | Individual still renders (isometric, hidden-line) of each frame |
| `leg_lengths.json` | Raw IK output (angles → leg lengths) for all frames |
| `wrist_model.py` | Parametric CadQuery source — edit parameters at the top and re-run to regenerate any pose, or add finer detail |

**Opening the STEP files:** Both SolidWorks (File → Open, select STEP,
import as parts/assembly) and Fusion 360 (Insert → Insert Mesh… no — use
Upload/Open, STEP imports natively) will read these directly as a multi-body
assembly with the base plate, platform plate, central ball joint, and 3 legs
(yoke – motor block – ball screw – nut – yoke) as separate bodies/components
you can then convert to native features, mate, and detail further.

## 6. Honest scope notes / what to refine next

- **U-joints and the central spherical joint are simplified stand-ins**
  (block-with-two-pins, and a plain sphere) — for a real build, swap in a
  COTS miniature universal joint (e.g., a small double-U/cardan joint) and a
  rod-end/spherical bearing for the central joint; the CAD anchor points and
  envelopes are already correctly positioned for that substitution.
- **Ball screw thread geometry is not modeled** (shown as a plain cylinder +
  nut block) — for manufacturing drawings, import the vendor's own SFU1204
  STEP model (most ball-screw suppliers publish one) into the same anchor
  locations produced here.
- **No structural/FEA sizing** has been done (plate thickness, bracket
  stress, screw column buckling at max extension) — recommended before
  cutting metal, especially if the wrist will carry any appreciable payload.
- Motor drivers/controller and wiring are out of scope of the mechanical
  package.
