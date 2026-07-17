# Omniverse Nut and Bolt Digital Twin
This repository explores two ways to model screw motion in a nut-and-bolt assembly inside NVIDIA Omniverse Isaac Sim:

1. **Driven bolt + rack-and-pinion joint** — the bolt rotates and a dedicated gear/screw constraint drives the nut.
2. **(NEW)  Fixed bolt + articulation mimic joint** — an intermediate slider is driven linearly, and a mimic joint makes the nut spin and travel.
---
> 🎬 **Coming Soon!** I'll publish a new video about the fixed bolt version and will update to the existing article with those additional details soon.
---

[Project Article](https://tech-multiverse.com/projects/how-to-create-a-nut-and-bolt-digital-twin-in-nvidia-omniverse-isaac-sim/)

[Project Video (rack-and-pinion)](https://youtu.be/sS73u0BHVP4)

[2 Minute YouTube Short Tutorial! (rack-and-pinion)](https://www.youtube.com/shorts/wFAfReHZda4)

<img src="_images/project_screenshot.png" alt="Project Screenshot" width="600"/>

## File Overview

* `official_hex_sim.usda` — **driven bolt using a rack-and-pinion joint.** The bolt is spun by a `PhysicsRevoluteJoint` with an angular drive, and the nut is coupled to it by a `PhysxPhysicsRackAndPinionJoint`. This is the original Tech-Multiverse example and the reliable approach when the bolt is the driven part.

* `static_bolt_spinning_nut.usda` — **(NEW)  fixed bolt using an articulation mimic joint.** The bolt is fixed to the world, an intermediate `NutSlide` link is driven by a `PhysicsPrismaticJoint`, and the visible nut follows through a `PhysxMimicJointAPI:rotZ`. This shows how to use mimic joints when the driven joint can be the prismatic leader.

* `/hex_bolt` — Contains the source CAD assets (.jt) and the USD representing the bolt and nut geometry, which are referenced in the main simulation.

* `/claude_sample` — A supplemental directory containing the Claude generated example Python script (`screw.py`) and the simulation USD (`claude_scripted.usda`) that example script created, which inspired this project!

---

## Versions of the Simulation
> ⚠️ **The Following is AI Generated**  
>Everything works, but I'm actively studying this and will update the documentation after I validate this information. Trust at your own risk! 😂

This repo contains two related USD setups. They produce different *visual* results, but they both rely on the same thread-pitch math. The important difference is which joint is driven and which physics constraint is used.

### 1. `official_hex_sim.usda` — bolt spins, nut travels (rack-and-pinion)

This is the version from the original Tech-Multiverse article and video.

* The **bolt** is connected to the world by a `PhysicsRevoluteJoint` and is spun by an angular drive (`drive:angular:physics:targetVelocity = 250`).
* The **nut** is connected to the world by a `PhysicsPrismaticJoint`, so it can only move up/down.
* A `PhysxPhysicsRackAndPinionJoint` couples the bolt's rotation to the nut's translation. Its `physics:ratio` (396,825) is `360 / pitch`, which means one full bolt revolution moves the nut by exactly the thread pitch.

**Why this works:** the rack-and-pinion joint is a *gear/screw* constraint. It is designed to turn a revolute degree of freedom into a prismatic degree of freedom at a fixed ratio, and it handles large ratios and high drive torques without fighting the drive. When the bolt spins, the rack-and-pinion constraint directly computes the matching nut velocity.

### 2. `static_bolt_spinning_nut.usda` — bolt fixed, nut spins and travels (mimic joint)

* The **bolt is fixed** to the world with a `PhysicsFixedJoint`.
* An intermediate `NutSlide` link is connected to the bolt by a prismatic joint.
* The visible **nut** is connected to `NutSlide` by a revolute joint.
* A `PhysxMimicJointAPI:rotZ` on the nut's revolute joint makes the nut spin as `NutSlide` translates.
* The linear drive is on the `NutSlidePrismatic` joint (`drive:linear:physics:targetVelocity = 0.00063`), not on the nut's revolute.

**Why it is set up this way:** `PhysxMimicJointAPI` is an articulation-level, position-based constraint that can only mimic *rotational* degrees of freedom (`rotX`, `rotY`, or `rotZ`). That means the follower joint must be a revolute, so the driven joint must be the prismatic *leader*. In this scene the bolt is fixed so it can act as the stationary guide, while the slider is driven and the nut follows as the revolute follower.

### Why not a mimic-joint version where the bolt spins?

We tried replacing the rack-and-pinion joint with `PhysxMimicJointAPI` in `official_hex_sim_no_rack_and_pinion.usda`. It did not work. `PhysxMimicJointAPI` is excellent for underactuated mechanisms (grippers, parallel linkages) but is not a true screw/gear joint. With a fine thread the gear ratio is very large (~400,000 deg/m), the reflected inertia becomes huge, and a velocity drive on the follower conflicts with the position-based mimic constraint. The result is that the nut drops or the simulation stalls. The rack-and-pinion joint remains the correct tool when the bolt is the driven part.

## Thread-pitch math

Both versions use the same screw relationship:

```text
nut_travel_per_revolution = pitch
linear_velocity           = angular_velocity (deg/s) * (pitch / 360)
```

* For the **rack-and-pinion** joint: `physics:ratio = 360 / pitch` (degrees per meter).
* For `PhysxMimicJointAPI`: `gearing = -(360 / pitch)`.

### Example: 28 TPI / ~0.907 mm pitch

```text
pitch = 0.0254 / 28 = 0.000907142857 m
pitch per degree = 0.000907142857 / 360 = 2.51984127e-6
```

### Example: M10-1.5 (1.5 mm pitch)

```text
pitch = 0.0015 m
pitch per degree = 0.0015 / 360 = 4.16666667e-6
```

If your bolt has a different pitch, replace the relevant constant (`ratio`, `gearing`, or `mimicCoef1`) with the value computed from the actual pitch.

## Tuning notes

* `drive:angular:physics:targetVelocity` or `drive:linear:physics:targetVelocity` changes only the *speed*, not the pitch ratio.
* `physics:lowerLimit` / `physics:upperLimit` on the prismatic joint controls how far the nut can travel *relative to its start pose*; they are not start-position values.
* To change the nut's starting height, move the `Nut` prim's `xformOp:translate` Z and set the prismatic joint's `physics:localPos0` Z to the same value (with `physics:lowerLimit = 0`).
* Self-collisions are disabled in the articulation-based files to prevent the nut and bolt collision meshes from fighting the screw constraint.

