# Wheeled Humanoid Robot (V2)

A custom-designed wheeled humanoid robot featuring a mobile base, dual humanoid arms, custom cycloidal drives, custom motor drivers, and integrated with ROS 2, Computer Vision, and AI.

## Project Structure

- **docs/**: Project documentation, communication protocols, and datasheets.
- **hardware/**: Mechanical CAD designs (SolidWorks/STEP) and electronic PCBs (KiCad).
- **firmware/**: Low-level MCU source code for controllers and motor drivers.
- **software/**: High-level ROS 2 packages and AI/Vision algorithms.
- **legacy/**: Submodule containing Version 1 of the AGV project (`agv_project`).

## Quick Start

### Submodule Initialization
If you just cloned this repository, initialize the legacy v1 submodule:
```bash
git submodule update --init --recursive
```
