# Reduced KUKA KR 16-2 Kinematics and Handwriting Simulation

MATLAB-based forward and inverse kinematics, workspace and singularity analysis, trajectory validation, and simulated handwriting for a reduced serial-chain model based on the KUKA KR 16-2 industrial manipulator.

> **Current model scope:** the supplied URDF and analytical implementation contain three active revolute joints and a fixed end-effector offset. This is a reduced kinematic model, not a complete six-axis KR 16-2 controller or dynamics model.

<!-- Replace this block after adding media/handwriting-demo.gif
<p align="center">
  <img src="media/handwriting-demo.gif" width="760" alt="Simulated handwriting trajectory">
</p>
-->

## Overview

This project develops and validates a position-level kinematic workflow for a reduced industrial manipulator model. The repository combines an analytical Denavit-Hartenberg formulation with URDF-based visualization in MATLAB.

The main workflow is:

1. Define a letter-shaped end-effector path in Cartesian space.
2. Check workspace feasibility and joint limits.
3. Solve closed-form inverse kinematics for elbow-up and elbow-down branches.
4. Validate the inverse-kinematics solutions using forward kinematics.
5. Animate the resulting joint trajectory on the URDF model.
6. Visualize joint variables, workspace projections, and singular configurations.

## Features

- Standard Denavit-Hartenberg forward kinematics.
- Closed-form inverse kinematics for a reduced 3R configuration.
- Elbow-up and elbow-down branch evaluation.
- Cartesian workspace validation and joint-limit checking.
- OXY and OXZ workspace visualization.
- Letter-stroke trajectory generation with pen-up transitions.
- URDF-based handwriting animation.
- Joint-variable and time-response plots.
- Visualization of elbow and radial singular configurations.
- STL mesh rebasing utility for aligning visual geometry with link frames.

## Repository structure

```text
.
├── README.md
├── setup_project.m
├── matlab/
│   ├── dhTransform.m
│   ├── fk_kuka3.m
│   ├── validate_letter_trajectory.m
│   ├── simulate_letter_writing.m
│   ├── workspace_visualize.m
│   ├── ik_visualize.m
│   ├── singularity_sim.m
│   ├── plot_joint_variables.m
│   ├── plot_time_response.m
│   └── ...
├── models/kuka_robot_model/
│   ├── urdf/
│   ├── meshes/
│   └── meshes_linkframe/
├── data/
├── results/
├── media/
└── docs/
```

## Requirements

- MATLAB
- Robotics System Toolbox

The mesh-preprocessing utility also requires MATLAB functions compatible with `stlread` and `stlwrite`.

Add the exact MATLAB release used for the final validated version:

```text
Tested with: MATLAB [ADD RELEASE]
```

## Quick start

Clone the repository and open MATLAB in the repository root.

```matlab
setup_project
```

Generate and validate the handwriting trajectory:

```matlab
validate_letter_trajectory
```

Animate the robot along the validated trajectory:

```matlab
simulate_letter_writing
```

The validation script creates:

```text
data/letter_trajectory_result.mat
```

## Additional analyses

### Forward-kinematics check

```matlab
test_fk_kuka3
```

### Inverse-kinematics branch visualization

```matlab
ik_visualize
```

### Workspace visualization

```matlab
workspace_visualize
```

### Singularity animation

```matlab
singularity_sim
```

### Joint-variable plots

Run trajectory validation first, then:

```matlab
plot_joint_variables
```

### Joint and end-effector time response

```matlab
plot_time_response
```

## Method summary

The reduced robot is represented using a Denavit-Hartenberg serial-chain model. Forward kinematics maps joint variables to the end-effector pose. The inverse-kinematics routine solves the two geometric elbow branches for each Cartesian path point.

Each trajectory point is accepted only if it:

- lies inside the analytical workspace,
- admits a finite inverse-kinematics solution, and
- satisfies the configured joint limits.

The validated joint sequence is then mapped to the URDF joint convention and animated using MATLAB Robotics System Toolbox.

## Results

Add the final public media here:

- **Handwriting simulation:** `[ADD VIDEO OR GIF LINK]`
- **Workspace analysis:** `[ADD IMAGE LINK]`
- **Inverse-kinematics branches:** `[ADD IMAGE LINK]`
- **Source demonstration:** `[ADD OPTIONAL FULL VIDEO LINK]`

## Current limitations

- The repository models three active revolute joints rather than the complete six-axis industrial manipulator.
- The current work evaluates position-level kinematics and trajectory feasibility, not physical tracking performance.
- Dynamics, actuator models, collision avoidance, and real-robot experiments are outside the current scope.
- Numerical parameters must be checked against the final CAD and URDF model before reporting quantitative results.

## Author

**Do Duc Nghia**  
Mechatronics Engineering Student  
Ho Chi Minh City University of Technology, VNU-HCM

- GitHub: <https://github.com/nghiado06>
- LinkedIn: <https://www.linkedin.com/in/nghiado265/>

## Citation

When referencing this academic project, use:

```bibtex
@software{do_duc_nghia_kuka_reduced_kinematics,
  author  = {Do Duc Nghia},
  title   = {Reduced KUKA KR 16-2 Kinematics and Handwriting Simulation},
  year    = {2026},
  url     = {https://github.com/nghiado06/kuka-kr16-2-reduced-kinematics}
}
```

## Notice

KUKA and KR 16-2 are used only for descriptive identification. This independent academic project is not affiliated with or endorsed by KUKA. Review `ASSET_NOTICE.md` before publishing the URDF and mesh files.
