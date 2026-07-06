# Omniverse Nut and Bolt Digital Twin
This repository provides a digital twin implementation of a nut-and-bolt assembly, designed for NVIDIA Omniverse Isaac Sim.

It leverages the rack and pinion joint to simulate linear motion from a rotational drive.

[Project Article](https://tech-multiverse.com/projects/how-to-create-a-nut-and-bolt-digital-twin-in-nvidia-omniverse-isaac-sim/)

[Project Video](https://youtu.be/sS73u0BHVP4)

[2 Minute YouTube Short Tutorial!](https://www.youtube.com/shorts/wFAfReHZda4)

<img src="_images/project_screenshot.png" alt="Project Screenshot" width="600"/>

## File Overview

* `official_hex_sim.usda`: The Universal Scene Description (USD) file serving as the final digital twin example. This file contains the stage hierarchy, prim properties, and physics constraints required to simulate the nut-and-bolt assembly within Omniverse Isaac Sim.

* `/hex_bolt`: Contains the source CAD assets (.jt) and the USD representing the bolt and nut geometry, which are referenced in the main simulation.

* `/claude_sample`: A supplemental directory containing the Claude generated example Python script (`screw.py`) and the simulation USD (`claude_scripted.usda`) that example script created.

