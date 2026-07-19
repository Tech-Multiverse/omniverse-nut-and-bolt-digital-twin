# Omniverse Nut and Bolt Digital Twin
This repository explores two ways to model screw motion in a nut-and-bolt assembly inside NVIDIA Omniverse Isaac Sim:

1. **Driven bolt + rack-and-pinion joint** — the bolt rotates and a dedicated gear/screw constraint drives the nut.
2. **Fixed bolt + articulation mimic joint** — an intermediate slider is driven linearly, and a mimic joint makes the nut spin and travel.

[Project Article](https://tech-multiverse.com/projects/how-to-create-a-nut-and-bolt-digital-twin-in-nvidia-omniverse-isaac-sim/)

[Project Video (rack-and-pinion)](https://youtu.be/sS73u0BHVP4)

[2 Minute YouTube Short Tutorial! (rack-and-pinion)](https://www.youtube.com/shorts/wFAfReHZda4)

---
> 🎬 **Coming Soon!** I'll publish a new video about the fixed bolt version soon.
---

<img src="_images/project_screenshot.png" alt="Project Screenshot" width="600"/>

## File Overview

* `rotating_bolt.usda` — **driven bolt using a rack-and-pinion joint.** The bolt is spun by a `PhysicsRevoluteJoint` with an angular drive, and the nut is coupled to it by a `PhysxPhysicsRackAndPinionJoint`.

* `static_bolt.usda` — **fixed bolt using an articulation mimic joint.** The bolt is fixed to the world, an intermediate `NutSlide` link is driven by a `PhysicsPrismaticJoint`, and the visible nut follows through a `PhysxMimicJointAPI`.

* `/hex_bolt` — Contains the source CAD assets (.jt) and the USD representing the bolt and nut geometry, which are referenced in the main simulation.

* `/claude_sample` — A supplemental directory containing the Claude generated example Python script (`screw.py`) and the simulation USD (`claude_scripted.usda`) created by that script, which inspired this project!

---

## Tuning notes

* `drive:angular:physics:targetVelocity` or `drive:linear:physics:targetVelocity` changes only the *speed*, not the pitch ratio.
* `physics:lowerLimit` / `physics:upperLimit` on the prismatic joint controls how far the nut can travel *relative to its start pose*; they are not start-position values.
* To change the nut's starting height, move the `Nut` prim's `xformOp:translate` Z and set the prismatic joint's `physics:localPos0` Z to the same value (with `physics:lowerLimit = 0`).
* Self-collisions are disabled in the articulation-based files to prevent the nut and bolt collision meshes from fighting the screw constraint.

