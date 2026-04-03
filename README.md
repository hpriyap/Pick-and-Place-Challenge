# 🦾 Pick-and-Place Challenge: Robotic Block Stacking with Static & Dynamic Targets

> A complete robotics pipeline combining AprilTag-based perception, iterative inverse kinematics, and turntable interception for autonomous block stacking on real hardware.

**University of Pennsylvania · December 2024**  
*Haiyue Chen · Wenjing Mao · Dixuan Lin · Hema Priya Pothumarthi*

---

## Overview

The Pick-and-Place Challenge required a robotic arm to autonomously detect, grasp, and stack blocks in two scenarios: picking **static blocks** from a table, and intercepting **moving blocks** from a continuously rotating turntable. The system integrates perception (AprilTag detection), motion planning (iterative IK), and real-time control — validated in both simulation and on physical hardware.

---

## Key Contributions

- **Detection Pipeline** — Built the full perception system for localizing blocks in 3D world coordinates using AprilTag detections. This includes multi-angle averaging across 12 camera configurations (3 distances × 4 angles) to reduce sensor noise, chained coordinate frame transforms ($M_w = M^w_{end} M^{end}_{cam} M_{det}$), and a custom rotation normalization algorithm that robustly extracts the z-axis yaw angle from any of the 5 visible AprilTag faces — including a circular-average method to correctly handle the 0°/90° wraparound discontinuity

- **Static Stage** — Implemented the full pipeline for detecting, grasping, and stacking static blocks: pose alignment via $R_{grasp} = R_y(\pi) \cdot R_z(\pi)$, pre-grasp approach 0.2m above each block, IK-based joint configuration computation, and layer-indexed stacking with precomputed target poses — achieving **100% grasp accuracy** and **zero misalignment** in both simulation and hardware

- **Dynamic Stage** — Designed the turntable interception strategy: computing each block's polar coordinates (radius $R$, angle $\theta$) relative to the turntable center, selecting the optimal target block (smallest $\theta$ with $0.22 < R < 0.3$ m), computing the grasp pose with a $\pm\frac{\pi}{4}$ y-axis end-effector tilt, and timing the intercept using $t_{wait} = \frac{\theta + \pi/4}{2\pi} \cdot t_{total}$ — achieving **75% dynamic grasp success** in simulation and successful hardware interception for the red team

---

## System Pipeline

```
┌─────────────────────────────────────────────────────────────┐
│                        STATIC STAGE                         │
│  Detect blocks → Align pose → IK grasp → Stack → Repeat     │
└─────────────────────────────────────────────────────────────┘
                              ↓ (no blocks remaining)
┌─────────────────────────────────────────────────────────────┐
│                       DYNAMIC STAGE                         │
│  Detect turntable → Select target (min θ, valid R)          │
│  → Compute grasp pose → Wait t_wait → Intercept             │
│  → Place on table → Treat as static → Stack                 │
└─────────────────────────────────────────────────────────────┘
```

---

## Methods

### Detection

Block poses are recovered by chaining three coordinate transforms:

$$M_w = M^w_{end} \cdot M^{end}_{cam} \cdot M_{det}$$

where $M_{det}$ is the 4×4 AprilTag pose, $M^{end}_{cam}$ is the camera-to-end-effector transform, and $M^w_{end}$ is obtained via forward kinematics. To handle AprilTag ambiguity (up to 5 visible faces per block), rotation matrices are normalized to extract a single z-axis yaw in [0°, 90°]. Averaging is performed using a circular difference accumulation method to avoid the 0°/90° wraparound error.

### Inverse Kinematics

An iterative IK solver computes joint configurations for target end-effector poses, using two Jacobian-based methods in sequence:

- **Jacobian Transpose:** $\dot{q}_{trans} = \alpha J^T e$
- **Jacobian Pseudoinverse:** $\dot{q}_{pseudo} = \alpha J^+ e$, where $J^+ = J^T(JJ^T)^{-1}$

A null-space joint-centering term keeps joints near their midrange:

$$\dot{q} = \dot{q}_{IK} + N\dot{q}_{center}, \quad N = I - J^+J$$

The solver falls back from Transpose to Pseudoinverse if convergence fails. Average IK computation time: **5.06 seconds**.

### Dynamic Grasping

Each turntable block's intercept parameters are computed as:

$$R = \sqrt{(x - x_0)^2 + (y - y_0)^2}, \quad \theta = \arccos\left(\frac{\mathbf{u} \cdot \mathbf{v}}{|\mathbf{u}||\mathbf{v}|}\right)$$

The robot pre-positions at the graspable radius and waits for the cube to arrive:

$$t_{wait} = \frac{\theta + \pi/4}{2\pi} \cdot t_{total}$$

The end-effector is tilted $\pm\frac{\pi}{4}$ around the y-axis to minimize collision risk with the turntable rim during interception.

---

## Results

### Simulation

| Metric | Result |
|--------|--------|
| Static cube detection accuracy | 100% |
| Static cube grasp success | 100% |
| Static cube stacking success | 100% (zero misalignment) |
| Dynamic cube rough detection | 100% |
| Dynamic cube grasp success | 75% (3/4 cubes) |
| Dynamic cube collision avoidance | 100% |

The single dynamic failure case occurred when a cube was oriented at a corner angle, leaving too little contact surface for the gripper to lift it.

### Hardware (Competition)

| Task | Red Team | Blue Team |
|------|----------|-----------|
| Static cubes stacked | 4/4 ✅ | 4/4 ✅ |
| Dynamic cube intercepted | 1/4 (placed, not stacked) | 0/4 ❌ |

Static stacking formed a near-perfect straight line on both teams, confirming the robustness of the detection and IK pipeline on real hardware. Dynamic failures on the blue team were traced to insufficient gripper height calibration for the physical turntable setup.

---

## Edge Cases Handled

**Missed block recovery** — If a block is missed during the first detection sweep (e.g., due to arena lighting), the system re-runs detection after the dynamic phase and picks up any remaining blocks automatically.

**Fallen cube filtering** — Cubes that fall off the tower could appear in the camera's field of view. A spatial filter rejects any detected block whose $(x, y)$ coordinates fall outside the source table boundary, preventing the robot from attempting to grasp floor-level objects.

---

## Lessons Learned

- Reasonable simplifying assumptions (turntable always moving, gravity for pose correction) greatly reduced planning complexity without sacrificing robustness
- Repeated multi-angle averaging was essential for reliable detection under real-world sensor noise
- Simulation-to-hardware gaps required careful per-axis offset recalibration; more hardware testing earlier would have improved the blue team's dynamic performance
- A hybrid IK + precomputed safety positions approach was significantly faster and safer than full motion planning for collision avoidance

---

## Demo Videos

| Setup | Link |
|-------|------|
| Simulation – Red Team | [Watch](https://drive.google.com/file/d/1I0TGP2uhvnIb1P_HtGzOORDO4bhsYZWr/view?usp=sharing) |
| Simulation – Blue Team | [Watch](https://drive.google.com/file/d/12BjSz0RJls_1owAijnqBBqw3tLhuY5CD/view?usp=sharing) |
| Hardware – Red Team | [Watch](https://drive.google.com/file/d/15IaIp7UrF5iTAY5oJxcQDspYuppz4Xgy/view?usp=drive_link) |
| Hardware – Blue Team | [Watch](https://drive.google.com/file/d/1Lb_9S-SbJ8Df9mfgYklJttZxKaG4tTrt/view?usp=drive_link) |

---

## Authors

| Name | Affiliation |
|------|-------------|
| Haiyue Chen | University of Pennsylvania |
| Wenjing Mao | University of Pennsylvania |
| Dixuan Lin | University of Pennsylvania |
| Hema Priya Pothumarthi | University of Pennsylvania |

---

*University of Pennsylvania · Robotics Pick-and-Place Challenge · December 2024*
