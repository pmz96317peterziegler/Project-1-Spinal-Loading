# Spinal & Hip Loading During a Lift — Inverse Kinematics + Statics

A 2D quasi-static biomechanical model that estimates the **erector-spinae (spinal) muscle force** and **hip joint reaction force** required to hold a load across a continuum of lifting postures. The body is modeled as a six-link planar chain; inverse kinematics generates the postures, and a static equilibrium analysis solves the internal loads at each one. The motivation is occupational: understanding how lower-back loading changes with posture during a lift.

---

## Overview

When someone lifts or holds an object, the lower-back muscles and the hip joint carry large internal loads that depend strongly on posture — a stooped lift loads the spine very differently from a squat. This project quantifies that relationship. It represents the body as six rigid links (foot, shank, thigh, trunk-head-neck, upper arm, and forearm-plus-hand), reaches a fixed target hand position through a range of postures, and at each posture solves the statics for:

- the **compressive** and **shear** components of force along the spine, and
- the net **spinal muscle force** and **hip joint reaction force**.

Results are reported against knee angle, which serves as a single scalar describing how squatted vs. upright the posture is.

## Method

**Anthropometry.** Segment lengths are derived from the subject's height using standard height-fraction landmarks, and segment masses, center-of-mass locations, and the body's mass distribution follow population anthropometric data. Because the model is a single sagittal-plane chain, the paired limbs (feet, shanks, thighs, arms) carry double mass to represent both sides, while the trunk-head-neck is counted once.

**Inverse kinematics (`compute_IK_solutions.m`).** The six-link chain reaching a 2D hand target is kinematically redundant — many joint configurations satisfy the same target. The solver builds a forward-kinematics model of the end-effector position and uses `fsolve` to drive it to the target. To sample a realistic family of solutions rather than one arbitrary pose, it seeds the solver with twenty initial guesses interpolated between two reference postures (a deep squat and a near-upright stance), producing twenty postures that smoothly span that range.

**Statics (`final_analysis.m`).** For each posture, homogeneous 4×4 transformation matrices chain the joint rotations and segment lengths to give the global position of every joint and segment center of mass, plus the whole-body center of mass (with and without the held object). A static equilibrium is then assembled and solved for the unknown hip reaction force and spinal muscle force. The hip force is projected onto the spine axis to separate **compression** (the component along the spine) from **shear** (the component across it). A stability test checks whether the body center of mass lies over the base of support, flagging each posture as stable, falling forward, or falling backward.

**Outputs.** Stick-figure plots of representative postures with all segment centers of mass marked, and summary curves of spinal force, shear force, and stability (compression) force against knee angle.

## Repository structure

```
.
├── src/
│   ├── final_analysis.m         # statics: spinal/hip forces, stability, plots
│   └── compute_IK_solutions.m   # inverse kinematics (forward model + fsolve)
└── README.md
```

## How to run

Requirements: **MATLAB** with the **Optimization Toolbox** (the inverse kinematics uses `fsolve`).

```matlab
% with src/ on the path
final_analysis   % runs IK + statics, prints a stability verdict per posture,
                 % and produces the posture and summary-force plots
```

`final_analysis.m` calls `compute_IK_solutions.m`, so both files must be on the MATLAB path. The target hand position (`x_target`, `y_target`), body height, body mass, and object mass are set at the top of `final_analysis.m`.

## Modeling assumptions & limitations

- **Planar (2D), symmetric.** Motion is confined to the sagittal plane; left and right limbs are lumped into single links with doubled mass.
- **Quasi-static.** Each posture is analyzed in static equilibrium; inertial and dynamic effects of the actual movement are not modeled.
- **Single equivalent spinal muscle.** The erector spinae is represented by one line of action with an assumed attachment geometry (a distal attachment ~75% up the trunk and a proximal attachment offset from the spine at the hip). These attachment assumptions are the model's largest source of uncertainty and dominate the spinal-force estimate.
- **Rigid links, idealized revolute joints**, with no antagonist co-contraction or passive tissue forces.
- **Population anthropometry** scaled from height and total mass rather than subject-specific measurement.
- **Redundant IK.** Because many postures reach the same target, the specific solutions returned depend on the two reference postures used to seed the solver.

## Possible extensions

- Replace the single-muscle assumption with a multi-muscle or antagonist model.
- Add inertial terms to extend from quasi-static to fully dynamic lifting.
- Sweep object mass and target location to map spinal load against lift parameters (an injury-risk surface).
- Subject-specific anthropometry and measured muscle attachment points.

## Authors

Group project: Nick Linkowski, Ben Brown, Peter Ziegler, Aidan Jones, Brady Stein, Bara Mbaye.
