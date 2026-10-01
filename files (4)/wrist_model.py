"""
3-UPU Spherical Wrist (pure rotation) — parametric generator
Architecture: 3 active UPU legs (ball-screw actuated prismatic) arranged
around a passive central spherical joint that fixes the platform's center
point O in space, so the moving platform can only ROTATE about O (3-DOF
spherical / pure-rotation motion), never translate.

Author: generated for wrist mechanism design task.
Units: mm
"""

import numpy as np
import cadquery as cq
from cadquery import exporters

# ---------------------------------------------------------------
# 1. DESIGN PARAMETERS
# ---------------------------------------------------------------
Rb      = 65.0      # radius of base U-joint circle
Rp      = 38.0      # radius of platform U-joint circle
Hc      = 70.0      # height of fixed center O above base plate
dz_p    = 8.0        # local platform-anchor offset above O (in platform frame)
zb_joint = 12.0       # height of base U-joint pivot above base plate

base_plate_r  = Rb + 20
base_plate_t  = 8
plat_plate_r  = Rp + 16
plat_plate_t  = 6

screw_dia   = 12.0   # SFU1204 ball screw nominal diameter
lead        = 4.0    # mm/rev
nut_len     = 22.0
nut_dia     = 24.0
motor_w     = 42.0   # NEMA17 body
motor_len   = 40.0
yoke_size   = 14.0
pin_dia     = 5.0
central_ball_r = 9.0
central_rod_r  = 5.0

# leg "home" extra length added on the base side before the screw starts
# (coupling + fixed yoke stub), purely geometric bookkeeping
base_stub = 18.0

# ---------------------------------------------------------------
# 2. KINEMATICS: anchor points & inverse kinematics
# ---------------------------------------------------------------
def base_points():
    pts = []
    for k in range(3):
        a = np.radians(90 + k * 120)   # phase, leg0 pointing to +Y
        pts.append(np.array([Rb * np.cos(a), Rb * np.sin(a), zb_joint]))
    return pts

def platform_points_local():
    pts = []
    for k in range(3):
        a = np.radians(90 + k * 120)
        pts.append(np.array([Rp * np.cos(a), Rp * np.sin(a), dz_p]))
    return pts

def rot_xyz(rx, ry, rz):
    rx, ry, rz = np.radians([rx, ry, rz])
    Rx = np.array([[1,0,0],[0,np.cos(rx),-np.sin(rx)],[0,np.sin(rx),np.cos(rx)]])
    Ry = np.array([[np.cos(ry),0,np.sin(ry)],[0,1,0],[-np.sin(ry),0,np.cos(ry)]])
    Rz = np.array([[np.cos(rz),-np.sin(rz),0],[np.sin(rz),np.cos(rz),0],[0,0,1]])
    return Rz @ Ry @ Rx

O = np.array([0, 0, Hc])

def platform_world_points(R):
    return [O + R @ p for p in platform_points_local()]

def leg_lengths(R):
    B = base_points()
    P = platform_world_points(R)
    return [np.linalg.norm(P[i] - B[i]) for i in range(3)], B, P

# ---------------------------------------------------------------
# 3. GEOMETRY BUILDERS
# ---------------------------------------------------------------
def yoke(size=yoke_size):
    """Simplified universal-joint block: two perpendicular pins in a block."""
    blk = cq.Workplane("XY").box(size, size, size*0.9)
    pin1 = cq.Workplane("XY").circle(pin_dia/2).extrude(size*1.3)\
            .rotate((0,0,0),(1,0,0),90).translate((0,0,0))
    pin2 = cq.Workplane("XY").circle(pin_dia/2).extrude(size*1.3)\
            .rotate((0,0,0),(0,1,0),90).translate((0,0,0))
    return blk.union(pin1).union(pin2)

def align_z_to(vec):
    """Return (rotation_axis, angle_deg) rotating +Z to vec."""
    z = np.array([0,0,1.0])
    v = vec/np.linalg.norm(vec)
    if np.allclose(v, z):
        return (1,0,0), 0.0
    if np.allclose(v, -z):
        return (1,0,0), 180.0
    axis = np.cross(z, v)
    axis = axis/np.linalg.norm(axis)
    angle = np.degrees(np.arccos(np.clip(np.dot(z, v), -1, 1)))
    return tuple(axis), angle

def leg_assembly(B, P, L):
    """Build one UPU leg (yoke-motor-screw-nut-yoke) from point B to point P."""
    direction = P - B
    axis, angle = align_z_to(direction)
    length = np.linalg.norm(direction)

    asm = cq.Assembly()

    # base yoke at B
    asm.add(yoke(), name="base_yoke",
            loc=cq.Location(cq.Vector(*B), cq.Vector(*axis), angle))

    # motor block sits just beyond base yoke, along leg direction
    motor = cq.Workplane("XY").box(motor_w, motor_w, motor_len)
    motor_center = B + (direction/length) * (yoke_size*0.6 + motor_len/2)
    asm.add(motor, name="motor",
            loc=cq.Location(cq.Vector(*motor_center), cq.Vector(*axis), angle))

    # ball screw shaft: from just after motor to just before platform yoke
    screw_start = yoke_size*0.6 + motor_len + 3
    screw_end = length - yoke_size*0.6
    screw_length = max(screw_end - screw_start, 5)
    screw = cq.Workplane("XY").circle(screw_dia/2).extrude(screw_length)
    screw_base_pt = B + (direction/length) * screw_start
    asm.add(screw, name="ballscrew",
            loc=cq.Location(cq.Vector(*screw_base_pt), cq.Vector(*axis), angle))

    # nut block: its position along the screw encodes the leg length L
    # (home position corresponds to screw_length/2; travel from there)
    nut_travel = screw_length * 0.5   # place near mid stroke for a "home" render
    nut = cq.Workplane("XY").circle(nut_dia/2).extrude(nut_len)
    nut_pt = screw_base_pt + (direction/length) * nut_travel
    asm.add(nut, name="nut",
            loc=cq.Location(cq.Vector(*nut_pt), cq.Vector(*axis), angle))

    # platform yoke at P
    asm.add(yoke(), name="platform_yoke",
            loc=cq.Location(cq.Vector(*P), cq.Vector(*axis), angle))

    return asm, length

def rotmat_to_axis_angle(R):
    """Convert a 3x3 rotation matrix to (axis, angle_deg). Handles near-identity."""
    angle = np.arccos(np.clip((np.trace(R) - 1) / 2.0, -1, 1))
    if angle < 1e-9:
        return (1, 0, 0), 0.0
    axis = np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]]) / (2*np.sin(angle))
    return tuple(axis), np.degrees(angle)

def build_wrist(rx=0, ry=0, rz=0, out_step=None, out_svg=None):
    R = rot_xyz(rx, ry, rz)
    Ls, B, P = leg_lengths(R)

    top = cq.Assembly()

    # base plate (fixed)
    base_plate = cq.Workplane("XY").circle(base_plate_r).extrude(base_plate_t)\
                    .translate((0,0,-base_plate_t))
    top.add(base_plate, name="base_plate")

    # platform plate (posed by R about O)
    plat_local = cq.Workplane("XY").circle(plat_plate_r).extrude(plat_plate_t)\
                    .translate((0,0,dz_p - plat_plate_t/2))
    axis, angle_deg = rotmat_to_axis_angle(R)
    plat_loc = cq.Location(cq.Vector(*O), cq.Vector(*axis), angle_deg)
    top.add(plat_local, name="platform_plate", loc=plat_loc)

    # central passive spherical joint (ball + stub rods), fixed at O
    ball = cq.Workplane("XY").sphere(central_ball_r).translate(tuple(O))
    rod_base = cq.Workplane("XY").circle(central_rod_r).extrude(Hc-4).translate((0,0,0))
    top.add(ball, name="central_ball")
    top.add(rod_base, name="central_rod_base")

    # 3 legs
    for i in range(3):
        leg_asm, Lact = leg_assembly(B[i], P[i], Ls[i])
        top.add(leg_asm, name=f"leg_{i}")

    if out_step:
        top.save(out_step, exportType="STEP")
    if out_svg:
        exporters.export(top.toCompound(), out_svg,
                          opt={"projectionDir": (1, -1, 0.6), "showHidden": False})
    return top, Ls

if __name__ == "__main__":
    import sys, json
    frames = [
        ("home",      0,  0,  0),
        ("tiltX15",  15,  0,  0),
        ("tiltY15",   0, 15,  0),
        ("combo",    10, 10,  6),
    ]
    results = {}
    for name, rx, ry, rz in frames:
        asm, Ls = build_wrist(rx, ry, rz,
                               out_step=f"/home/claude/wrist_{name}.step",
                               out_svg=f"/home/claude/wrist_{name}.svg")
        results[name] = {"angles_deg": [rx, ry, rz], "leg_lengths_mm": [round(l,3) for l in Ls]}
        print(name, results[name])
    with open("/home/claude/leg_lengths.json","w") as f:
        json.dump(results, f, indent=2)
