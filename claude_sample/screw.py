"""
Bolt-and-nut (Schraubverbindung) using PhysxPhysicsRackAndPinionJoint.

>>> Run this INSIDE Isaac Sim, in the Script Editor window
    (Window -> Script Editor), NOT via ./python.sh <<<

A RackAndPinionJoint normally couples a rotating pinion to a sliding rack.
Here both the revolute axis (bolt) and the prismatic axis (nut) point along Z,
so rotation converts to translation on the same axis — exactly like a screw thread.

PhysX constraint (empirically verified):
    omega [deg/s]  =  V [m/s]  *  ratio
    → ratio = 360 / pitch_m    (degrees per meter of travel per revolution)

Example:  pitch = 2 cm = 0.02 m  →  ratio = 360 / 0.02 = 18 000

Unlike the standalone version this script:
  * does NOT create a SimulationApp (the app is already running)
  * builds the scene on the currently-open stage
  * runs the simulation through the Timeline + an async monitor task,
    so the editor UI stays responsive.
"""

import math

import asyncio

import omni.usd
import omni.timeline
import omni.kit.app
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

# ---------------------------------------------------------------------------
# Parameters  (SI units: meters)
# ---------------------------------------------------------------------------
THREAD_PITCH_M   = 0.02     # meters per revolution (2 cm pitch)
BOLT_SPEED_DEG_S = 90.0     # angular drive on bolt [deg/s]
SIM_DURATION_S   = 8.0      # how long the async monitor reports for

BOLT_RADIUS      = 0.015    # m
BOLT_HEIGHT      = 0.30     # m
NUT_OUTER_RADIUS = 0.035    # m
NUT_HEIGHT       = 0.04     # m
NUT_START_Z      = 0.02     # m — nut starting position

# PhysX constraint:  omega [deg/s] = V [m/s] * ratio
ratio = 360.0 / THREAD_PITCH_M                  # 18 000 for 2 cm pitch
EXPECTED_NUT_SPEED = BOLT_SPEED_DEG_S / ratio   # m/s

# ---------------------------------------------------------------------------
# Use the currently-open stage (created by the running app)
# ---------------------------------------------------------------------------
stage = omni.usd.get_context().get_stage()

# A "/World" scope to keep things tidy
if not stage.GetPrimAtPath("/World"):
    UsdGeom.Xform.Define(stage, "/World")

# Remove any leftovers from a previous run so the script is re-runnable
for path in ["/World/Bolt", "/World/Nut", "/World/Ground",
             "/World/BoltRevoluteJoint", "/World/NutPrismaticJoint",
             "/World/ScrewJoint", "/World/PhysicsScene"]:
    if stage.GetPrimAtPath(path):
        stage.RemovePrim(path)

# Physics scene (the running app does not create one automatically)
scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/PhysicsScene"))
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
scene.CreateGravityMagnitudeAttr().Set(9.81)   # m/s²  (stage units = meters)

# ---------------------------------------------------------------------------
# Ground plane (USD geometry, no nucleus assets)
# ---------------------------------------------------------------------------
gnd = UsdGeom.Xform.Define(stage, "/World/Ground")
gnd.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.005))
gnd_box = UsdGeom.Cube.Define(stage, "/World/Ground/Shape")
gnd_box.CreateSizeAttr(1.0)
gnd_box.AddScaleOp().Set(Gf.Vec3f(2.0, 2.0, 0.01))
gnd_box.CreateDisplayColorAttr().Set([Gf.Vec3f(0.3, 0.3, 0.3)])
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath("/World/Ground/Shape"))

# ---------------------------------------------------------------------------
# Helper: cylinder rigid body (visual-only, no contact geometry needed)
# ---------------------------------------------------------------------------
def make_cylinder(path, radius, height, z_center, color, mass):
    p = Sdf.Path(path)
    xf = UsdGeom.Xform.Define(stage, p)
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, z_center))
    cyl = UsdGeom.Cylinder.Define(stage, p.AppendPath("Shape"))
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(height)
    cyl.CreateAxisAttr("Z")
    cyl.CreateDisplayColorAttr().Set([color])
    prim = stage.GetPrimAtPath(p)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    return p

# ---------------------------------------------------------------------------
# Bolt  – rotates around Z, anchored by RevoluteJoint to world
# ---------------------------------------------------------------------------
bolt_z = BOLT_HEIGHT / 2.0
bolt_path = make_cylinder(
    "/World/Bolt", BOLT_RADIUS, BOLT_HEIGHT, bolt_z,
    Gf.Vec3f(0.85, 0.65, 0.1), mass=0.5,   # gold
)

bolt_joint_path = Sdf.Path("/World/BoltRevoluteJoint")
bolt_joint = UsdPhysics.RevoluteJoint.Define(stage, bolt_joint_path)
bolt_joint.CreateAxisAttr("Z")
bolt_joint.CreateBody1Rel().SetTargets([bolt_path])   # body0 absent = world
bolt_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, bolt_z))
bolt_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
bolt_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
bolt_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

# Angular velocity drive — spin the bolt like a powered screw
drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(bolt_joint_path), "angular")
drive.CreateTypeAttr("force")
drive.CreateTargetVelocityAttr(BOLT_SPEED_DEG_S)
drive.CreateDampingAttr(1e6)
drive.CreateStiffnessAttr(0.0)

# ---------------------------------------------------------------------------
# Nut  – slides along Z, blocked from rotation by PrismaticJoint to world
# ---------------------------------------------------------------------------
nut_travel_max = BOLT_HEIGHT - NUT_HEIGHT - NUT_START_Z    # 0.24 m
nut_path = make_cylinder(
    "/World/Nut", NUT_OUTER_RADIUS, NUT_HEIGHT, NUT_START_Z,
    Gf.Vec3f(0.4, 0.5, 0.8), mass=0.1,     # steel blue
)

nut_joint_path = Sdf.Path("/World/NutPrismaticJoint")
nut_joint = UsdPhysics.PrismaticJoint.Define(stage, nut_joint_path)
nut_joint.CreateAxisAttr("Z")
nut_joint.CreateBody1Rel().SetTargets([nut_path])     # body0 absent = world
nut_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, NUT_START_Z))
nut_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
nut_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
nut_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
nut_joint.CreateLowerLimitAttr(0.0)
nut_joint.CreateUpperLimitAttr(nut_travel_max)

# ---------------------------------------------------------------------------
# Screw coupling: RackAndPinionJoint with rotation AND translation along Z
# ---------------------------------------------------------------------------
screw = PhysxSchema.PhysxPhysicsRackAndPinionJoint.Define(
    stage, Sdf.Path("/World/ScrewJoint")
)
screw.CreateBody0Rel().SetTargets([bolt_path])
screw.CreateBody1Rel().SetTargets([nut_path])
screw.CreateHingeRel().SetTargets([bolt_joint_path])
screw.CreatePrismaticRel().SetTargets([nut_joint_path])
screw.CreateRatioAttr(ratio)

# ---------------------------------------------------------------------------
# Start playback and monitor the nut asynchronously (keeps the UI responsive)
# ---------------------------------------------------------------------------
print()
print("=" * 64)
print("  Bolt-Nut Screw Joint  (PhysxPhysicsRackAndPinionJoint)")
print("=" * 64)
print(f"  Thread pitch  : {THREAD_PITCH_M*100:.1f} cm per revolution")
print(f"  Bolt speed    : {BOLT_SPEED_DEG_S} deg/s  ({BOLT_SPEED_DEG_S/360:.3f} rev/s)")
print(f"  Ratio         : {ratio:.1f}  (360 / pitch_m)")
print(f"  Expected nut  : {EXPECTED_NUT_SPEED*1000:.4f} mm/s")
print(f"  Max travel    : {nut_travel_max*100:.1f} cm")
print("  Scene built. Starting timeline...")


async def _monitor_screw():
    timeline = omni.timeline.get_timeline_interface()
    app = omni.kit.app.get_app()
    nut_xf = UsdGeom.Xform(stage.GetPrimAtPath(nut_path))

    # Begin playback
    timeline.play()
    await app.next_update_async()  # let one frame tick so physics initialises

    z0 = nut_xf.ComputeLocalToWorldTransform(0).ExtractTranslation()[2]
    t0 = timeline.get_current_time()

    print(f"  Baseline Z    : {z0*1000:.3f} mm")
    print()
    print(f"  {'t [s]':>7}  {'Nut Z [mm]':>11}  {'Disp [mm]':>10}  {'Expected [mm]':>13}")
    print("  " + "-" * 48)

    next_report = 0.0
    while True:
        t = timeline.get_current_time() - t0
        if t >= next_report:
            nut_z = nut_xf.ComputeLocalToWorldTransform(0).ExtractTranslation()[2]
            disp  = (nut_z - z0) * 1000.0
            exp_d = min(EXPECTED_NUT_SPEED * t, nut_travel_max) * 1000.0
            print(f"  {t:>7.2f}  {nut_z*1000:>11.3f}  {disp:>+10.3f}  {exp_d:>13.3f}")
            next_report += 1.0
        if t >= SIM_DURATION_S:
            break
        await app.next_update_async()

    print()
    print("  Simulation complete. (timeline still playing — press Stop to reset)")


# Schedule the monitor on the app's event loop (Script Editor is async-friendly)
asyncio.ensure_future(_monitor_screw())
