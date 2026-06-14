# Vista-Tracker

Vista-Tracker presents a trajectory planning framework for autonomous aerial
tracking of a moving target in cluttered environments. The approach focuses on
maintaining target visibility while ensuring safe and dynamically feasible
quadrotor motion for applications such as aerial cinematography, inspection,
and search and rescue.

## Overview

The framework addresses the coupled challenges of perception-aware planning,
obstacle avoidance, and real-time trajectory optimization through a two-stage
motion-planning architecture:

- **Front-end:** generates a reference trajectory for target tracking by
  modeling the quadrotor-target interaction as a leader-follower problem.
- **Back-end:** uses a Model Predictive Control (MPC) formulation to compute
  smooth, feasible, and safe trajectories that respect nonlinear quadrotor
  dynamics and actuator limits.

## MPC Objectives

The MPC formulation combines multiple soft objectives to support robust aerial
tracking:

- maintain a desired distance to the target
- minimize field-of-view (FOV) occlusion
- enforce collision avoidance through a repulsive potential cost
- align the quadrotor FOV with the target

## Evaluation

The method is evaluated in simulation across a broad set of scenarios,
including:

- tracking-behavior verification
- obstacle-avoidance experiments focused on the collision-avoidance cost
- dense cluttered environments that test visibility maintenance in constrained
  spaces
- sensitivity analyses for cost weights, prediction horizon, and noisy target
  measurements

## Results

The reported results show that the proposed approach consistently achieves
collision-free tracking while maintaining target visibility by reducing
occlusion, even in cluttered environments. The MPC solver also reaches
computation times suitable for online execution, supporting real-time
autonomous tracking.