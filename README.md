

# UAV Trajectory Tracking Controller ( Code in french)

This repository contains the MATLAB/Simulink implementation of a 3‑DOF feedback linearisation controller for UAV trajectory tracking. The controller was designed to track a Bézier curve trajectory and validated on a physical robot.

## Project Overview

- **Control Strategy**: Feedback linearisation to handle the non‑linear dynamics of the UAV; differential flatness; MPC
- **Trajectory**: Bézier curve, generated from a set of control points.
- **Gain Synthesis**: PID gains derived analytically from closed‑loop specifications (rise time, settling time, overshoot).
- **Validation**: The controller was first simulated in Simulink and then successfully tested on a physical robot, confirming stability and tracking precision.

## Repository Contents

| File | Description |
|------|-------------|
| `UAVFeedbackLinearize.slx` | Main Simulink model implementing the feedback linearisation controller. |
| `UAVFeedbackLinearize.slx.r2019a` | Simulink model saved in a format compatible with MATLAB R2019a. |
| `PIDcontSynth_v8_simplifieV3.p` | MATLAB script for analytical PID gain synthesis (protected P‑file). |
| `bezier_curve.m` | Function to generate a Bézier curve from a set of control points. |
| `compute_bezier_control_points.m` | Helper script to compute the required control points for the reference trajectory. |
| `IP_plotresults_UAV.m` | Post‑processing script to plot and analyse simulation/experiment results. |
| `TP1_AC470.m` / `TP1_AC470.mlx` | Practical work 1 – introductory UAV modelling. |
| `TP2_fcn.m` / `TP2_fcn2.m` / `TP2_Doc.pdf` / `TP2_Subject.pdf` | Practical work 2 – controller design steps and supporting functions. |
| `TP3.m` / `TP3_Subject.pdf` | Practical work 3 – advanced trajectory tracking. |
| `*.pdf` files | Subject descriptions and documentation for the practical work sessions. |

## Requirements

- **MATLAB** (R2019a or later recommended)
- **Simulink** (including the following toolboxes):
  - Simulink Control Design
  - Aerospace Toolbox (optional, for UAV modelling)
- No additional third‑party libraries are required.

## Getting Started

1. **Clone the repository**:
   ```bash
   git clone https://github.com/C0ulday/UAV-Control.git
