---
id: chapter-02
title: Chapter 2
sidebar_label: Chapter 2
---

### Chapter 02: Robot Kinematics

#### Learning Objectives
After studying this chapter, you should be able to:
*   Differentiate rigorously between Forward Kinematics (FK) and Inverse Kinematics (IK), including their computational complexities and common applications.
*   Systematically apply the Denavit-Hartenberg (DH) convention to establish coordinate frames and derive kinematic parameters for multi-link robot manipulators.
*   Construct and compose homogeneous transformation matrices to represent the position and orientation of robot links and end-effectors in 3D space.
*   Derive and interpret the Jacobian matrix for robotic systems, understanding its role in velocity mapping, singularity analysis, and static force analysis.

#### Theory Explanation

**2.1 Introduction to Robot Kinematics**

**Robot Kinematics** is a foundational branch of robotics that deals with the study of motion without considering the forces and torques that cause the motion. It focuses on the geometric relationships between the robot's joint variables (e.g., angles of revolute joints, linear displacements of prismatic joints) and the position and orientation of its end-effector or any point of interest in its workspace. A thorough understanding of kinematics is prerequisite for dynamics, control, and path planning.

**2.2 Forward Kinematics (FK)**

**Forward Kinematics (FK)** is the direct mapping from the robot's joint space to its task space. Given the values of all joint variables, FK calculates the resulting position and orientation of the end-effector relative to a fixed base coordinate frame. This is a deterministic and usually straightforward computation, typically involving sequential multiplication of transformation matrices.

For an `n`-DOF robot, if `q = [q1, q2, ..., qn]^T` represents the vector of joint variables, Forward Kinematics computes `X = f(q)`, where `X` is the end-effector's pose (position and orientation) in Cartesian space.

**Applications of FK include:**
*   Visualizing the robot's configuration given joint commands.
*   Determining if a specific joint configuration causes a collision.
*   Generating sensor expectations (e.g., where a camera should be pointing).

**2.3 Inverse Kinematics (IK)**

**Inverse Kinematics (IK)** is the inverse problem of FK: given a desired position and orientation of the end-effector in task space, IK determines the corresponding set of joint variables (`q`) that can achieve that pose. This is generally a much more complex and computationally intensive problem than FK due to several factors:

*   **Non-linearity**: The relationship between joint variables and end-effector pose is often non-linear, especially for robots with many degrees of freedom.
*   **Multiple Solutions**: A desired end-effector pose may be reachable by several different joint configurations (e.g., an "elbow up" or "elbow down" solution for a human arm-like manipulator).
*   **No Solutions**: The desired pose might be outside the robot's workspace, meaning no valid joint configuration exists.
*   **Singularities**: Certain joint configurations (singularities) can lead to a loss of degrees of freedom in the task space, making IK difficult or impossible to solve at those points.

IK can be solved using analytical methods (for simpler robots) or numerical/iterative methods (for complex or redundant robots). Numerical methods often involve Jacobian-based approaches (discussed below).

**Applications of IK include:**
*   Task-space control, where a robot needs to precisely follow a path or trajectory in Cartesian space (e.g., welding, painting, grasping).
*   Human-robot collaboration, where the robot mimics human movements.
*   Path planning, where IK is used to convert desired end-effector waypoints into executable joint trajectories.

**2.4 Denavit-Hartenberg (DH) Convention**

The **Denavit-Hartenberg (DH) Convention** provides a systematic and standardized method for assigning coordinate frames to each link of a serial robot manipulator and for deriving the transformation matrices between adjacent links. This convention simplifies the derivation of the forward kinematics equation for complex robotic arms.

For each link `i` in a robot chain, a coordinate frame `{i}` is attached. The transformation from frame `{i-1}` to frame `{i}` is defined by four parameters:

*   **`-i` (Link Length)**: The distance along the common normal between the `z_{i-1}` and `z_i` axes. It is the shortest distance between the two joint axes.
*   **`α_i` (Link Twist)**: The angle from the `z_{i-1}` axis to the `z_i` axis, measured about the common normal. It describes how much the `z_i` axis is "twisted" relative to `z_{i-1}`.
*   **`d_i` (Link Offset)**: The distance along the `z_{i-1}` axis from the origin of frame `{i-1}` to the intersection of the `z_{i-1}` axis with the common normal. For a revolute joint, this is typically constant. For a prismatic joint, `d_i` is the joint variable.
*   **`θ_i` (Joint Angle)**: The angle from the `x_{i-1}` axis to the `x_i` axis, measured about the `z_{i-1}` axis. For a revolute joint, this is the joint variable. For a prismatic joint, `θ_i` is typically constant.

The general homogeneous transformation matrix `A_i^{i-1}` from frame `{i-1}` to frame `{i}` using DH parameters is:

```
A_{i}^{i-1} =
  Rot(z_{i-1}, \thet-i) \cdot Trans(z_{i-1}, d_i) \cdot Trans(x_i, -i) \cdot Rot(x_i, \alph-i)

= [[cos(\thet-i), -sin(\thet-i)cos(\alph-i), sin(\thet-i)sin(\alph-i), -i cos(\thet-i)],
   [sin(\thet-i),  cos(\thet-i)cos(\alph-i), -cos(\thet-i)sin(\alph-i), -i sin(\thet-i)],
   [0,               sin(\alph-i),             cos(\alph-i),             d_i],
   [0,               0,                         0,                         1]]
```

The full forward kinematics from the base frame `{0}` to the end-effector frame `{N}` is then the product of these successive transformation matrices:

`T_N^0 = A_1^0 A_2^1 ... A_N^{N-1}`

**2.5 Homogeneous Transformation Matrices**

**Homogeneous Transformation Matrices** (HTMs) are 4x4 matrices that combine both rotation and translation into a single mathematical entity. They provide a concise and powerful way to represent the pose (position and orientation) of one coordinate frame relative to another in 3D Euclidean space. A general HTM `T` is structured as:

```
T = [[R_{3x3}, p_{3x1}],
     [0_{1x3}, 1_{1x1}]]
```

Where:
*   `R_{3x3}` is a 3x3 rotation matrix, representing the orientation of the new frame's axes with respect to the reference frame.
*   `p_{3x1}` is a 3x1 position vector, representing the origin of the new frame with respect to the reference frame.
*   `0_{1x3}` is a 1x3 zero vector.
*   `1_{1x1}` is a scalar one.

HTMs allow for sequential transformations to be composed by matrix multiplication: if `T_B^A` transforms points from frame `{B}` to frame `{A}`, and `T_C^B` transforms points from frame `{C}` to frame `{B}`, then `T_C^A = T_B^A T_C^B` transforms points from `{C}` to `{A}`.

**2.6 The Jacobian Matrix**

The **Jacobian Matrix** (`J`) in robotics is a fundamental tool that relates velocities in the robot's joint space to velocities in its task space (Cartesian space). Specifically, it maps joint velocities (`q̇`) to the linear (`v`) and angular (`ω`) velocities of the end-effector:

`[v; ω] = J(q) * q̇`

Where:
*   `J(q)` is the `6 x n` Jacobian matrix, dependent on the current joint configuration `q`.
*   `q̇` is the `n x 1` vector of joint velocities.
*   `[v; ω]` is the `6 x 1` vector representing the end-effector's linear and angular velocities.

**Key applications and insights from the Jacobian:**

*   **Velocity Analysis**: Directly computes end-effector velocities from joint velocities.
*   **Inverse Kinematics**: For redundant robots or in numerical IK methods, `q̇ = J⁺(q) * [v; ω]` (where `J⁺` is the pseudo-inverse) can be used to compute joint velocities to achieve desired end-effector velocities.
*   **Singularity Analysis**: Robot singularities occur when the Jacobian matrix loses rank (i.e., its determinant is zero, or it does not have full rank). At these configurations, the robot effectively loses one or more degrees of freedom in its task space, meaning it cannot move its end-effector in certain directions despite non-zero joint velocities. Singularities are critical to avoid in robot operation.
*   **Static Force Analysis**: The transpose of the Jacobian (`J^T`) relates forces/torques in task space to torques in joint space (e.g., `τ = J^T * F`), which is important for understanding wrench propagation and control.

#### Diagrams

```mermaid
graph TD
    subgraph Robot Kinematics
        FK[Forward Kinematics] -- q --> EE_Pose(End-Effector Pose);
        EE_Pose -- X --> IK[Inverse Kinematics];
        IK -- q --> FK;

        subgraph DH Parameters
            L1(Link 1 DH) --> TM1(Transform Matrix 1);
            L2(Link 2 DH) --> TM2(Transform Matrix 2);
            TM1 --> TM_COMPOSITION(Composition of HTMs);
            TM2 --> TM_COMPOSITION;
            TM_COMPOSITION --> EE_Pose;
        end

        subgraph Jacobian Analysis
            JV(Joint Velocities q_dot) --> JM(Jacobian Matrix J(q));
            JM --> TV(Task Space Velocities [v, omega]);
            JM --> Singularities(Singularity Analysis);
        end
    end
```
**Figure 2.1: Integrated View of Robot Kinematics Concepts**

```mermaid
graph TD
    Joint1(Joint 1 Angle) --> Link1(Link 1);
    Joint2(Joint 2 Angle) --> Link2(Link 2);
    Link1 & Link2 --> EndEffector(End-Effector Position/Orientation);

    style Joint1 fill:#fff,stroke:#333,stroke-width:1px;
    style Link1 fill:#DCEFFB,stroke:#333,stroke-width:1px;
    style Joint2 fill:#fff,stroke:#333,stroke-width:1px;
    style Link2 fill:#DCEFFB,stroke:#333,stroke-width:1px;
    style EndEffector fill:#F9F,stroke:#333,stroke-width:2px;

    subgraph Forward Kinematics (FK)
        FK_Input(Joint Angles) --> FK_Process(Geometric Calculations) --> FK_Output(End-Effector Pose);
    end

    subgraph Inverse Kinematics (IK)
        IK_Input(Desired End-Effector Pose) --> IK_Process(Solver Algorithm) --> IK_Output(Joint Angles);
    end

    FK_Output -- Used by --> IK_Input;
    IK_Output -- Controls --> FK_Input;
```
**Figure 2.2: Forward vs. Inverse Kinematics Relationship**

#### Python/ROS2 Code Examples

##### 1. Homogeneous Transformation and 2D Forward Kinematics (Python)
This expanded Python example provides functions for 2D rotations, translations, and constructing homogeneous transformation matrices. It then applies these to a 2-DOF planar arm to calculate its forward kinematics.

```python
import numpy as np
import math

def rot_z(angle_rad):
    """Generates a 2D rotation matrix around the Z-axis (for 3D context, this is a Z-rotation)."""
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]])

def trans_xyz(x, y, z):
    """Generates a 3D translation vector."""
    return np.array([x, y, z])

def homogeneous_transform_matrix(rotation_matrix, translation_vector):
    """Constructs a 4x4 homogeneous transformation matrix.
    rotation_matrix: 3x3 numpy array
    translation_vector: 3x1 numpy array
    """
    T = np.eye(4)
    T[:3, :3] = rotation_matrix
    T[:3, 3] = translation_vector
    return T

def forward_kinematics_2d_planar_arm(L1, L2, theta1_rad, theta2_rad):
    """Calculates the end-effector position (x, y) for a 2-DOF planar arm.
    L1, L2: Link lengths
    theta1_rad, theta2_rad: Joint angles in radians (from the previous link or base)
    """
    # Position of joint 1 (relative to base, which is origin)
    x1 = L1 * math.cos(theta1_rad)
    y1 = L1 * math.sin(theta1_rad)

    # Position of end-effector (relative to base)
    # The angle for the second link is relative to the first link
    x_ee = x1 + L2 * math.cos(theta1_rad + theta2_rad)
    y_ee = y1 + L2 * math.sin(theta1_rad + theta2_rad)

    return x_ee, y_ee

if __name__ == "__main__":
    # Example: 2-DOF planar arm
    link1_length = 1.0  # meters
    link2_length = 0.8  # meters
    joint1_angle = math.pi / 6  # 30 degrees
    joint2_angle = math.pi / 4  # 45 degrees

    ee_x, ee_y = forward_kinematics_2d_planar_arm(link1_length, link2_length, joint1_angle, joint2_angle)
    print(f"End-effector position for 2-DOF arm: ({ee_x:.3f}, {ee_y:.3f}) meters")

    # Example: Constructing and composing Homogeneous Transformation Matrices
    # Transformation from Frame 0 to Frame 1 (rotation + translation)
    R0_1 = rot_z(math.pi / 4) # Rotate 45 deg around Z
    p0_1 = trans_xyz(0.5, 0.2, 0.0) # Translate by (0.5, 0.2, 0)
    T0_1 = homogeneous_transform_matrix(R0_1, p0_1)
    print("\nTransformation T0_1 (Frame 0 to Frame 1):")
    print(T0_1)

    # Transformation from Frame 1 to Frame 2 (only translation)
    R1_2 = np.eye(3) # No rotation
    p1_2 = trans_xyz(0.3, 0.0, 0.0) # Translate by (0.3, 0, 0) along its own X
    T1_2 = homogeneous_transform_matrix(R1_2, p1_2)
    print("\nTransformation T1_2 (Frame 1 to Frame 2):")
    print(T1_2)

    # Compose transformations: T0_2 = T0_1 * T1_2
    T0_2 = np.dot(T0_1, T1_2)
    print("\nComposed Transformation T0_2 (Frame 0 to Frame 2):")
    print(T0_2)

    # Extract position and orientation from T0_2
    final_position = T0_2[:3, 3]
    final_orientation_matrix = T0_2[:3, :3]
    print(f"\nFinal Position of Frame 2 wrt Frame 0: {final_position}")
    print(f"Final Orientation Matrix of Frame 2 wrt Frame 0:\n{final_orientation_matrix}")
```

##### 2. Conceptual Jacobian for a 2-DOF Planar Arm (Python)
This example conceptually derives and shows the structure of a Jacobian for a simple 2-DOF planar arm, relating joint angular velocities to end-effector linear velocities. A full analytical derivation would be more involved but this captures the essence.

```python
import numpy as np
import math

def calculate_jacobian_2d_planar_arm(L1, L2, theta1_rad, theta2_rad):
    """Calculates the Jacobian matrix for a 2-DOF planar arm.
    J relates [theta1_dot, theta2_dot] to [x_dot, y_dot].
    """
    # Partial derivatives of x_ee and y_ee with respect to theta1 and theta2
    # x_ee = L1 * cos(theta1) + L2 * cos(theta1 + theta2)
    # y_ee = L1 * sin(theta1) + L2 * sin(theta1 + theta2)

    # dx_ee/d(theta1)
    J11 = -L1 * math.sin(theta1_rad) - L2 * math.sin(theta1_rad + theta2_rad)
    # dx_ee/d(theta2)
    J12 = -L2 * math.sin(theta1_rad + theta2_rad)

    # dy_ee/d(theta1)
    J21 = L1 * math.cos(theta1_rad) + L2 * math.cos(theta1_rad + theta2_rad)
    # dy_ee/d(theta2)
    J22 = L2 * math.cos(theta1_rad + theta2_rad)

    Jacobian = np.array([
        [J11, J12],
        [J21, J22]
    ])
    return Jacobian

if __name__ == "__main__":
    link1_length = 1.0  # meters
    link2_length = 0.8  # meters
    joint1_angle = math.pi / 6  # 30 degrees
    joint2_angle = math.pi / 4  # 45 degrees

    J_arm = calculate_jacobian_2d_planar_arm(link1_length, link2_length, joint1_angle, joint2_angle)
    print("Jacobian Matrix for 2-DOF Planar Arm:")
    print(np.array2string(J_arm, precision=3, separator=',', suppress_small=True))

    # Example: Calculate end-effector velocity for given joint velocities
    joint_velocities = np.array([0.1, 0.2]) # rad/s for theta1_dot, theta2_dot
    ee_velocities = np.dot(J_arm, joint_velocities)
    print(f"\nJoint Velocities (rad/s): {joint_velocities}")
    print(f"End-effector Velocities (m/s): {ee_velocities}")

    # Singularity Check: determinant of Jacobian is zero
    det_J = np.linalg.det(J_arm)
    print(f"\nDeterminant of Jacobian: {det_J:.3f}")
    if abs(det_J) < 1e-6: # Check if determinant is close to zero
        print("Warning: Robot is near a kinematic singularity!")
    else:
        print("Robot is in a non-singular configuration.")

    # Example of a singular configuration (arm fully extended or folded)
    print("\n--- Testing a Singular Configuration ---")
    singular_J = calculate_jacobian_2d_planar_arm(1.0, 1.0, 0.0, 0.0) # Fully extended
    print("Jacobian at singular config (theta1=0, theta2=0):")
    print(np.array2string(singular_J, precision=3, separator=',', suppress_small=True))
    print(f"Determinant: {np.linalg.det(singular_J):.3f}")
    if abs(np.linalg.det(singular_J)) < 1e-6:
        print("Correctly identified singularity.")
```

#### Exercises + MCQs

##### Exercises
1.  **DH Parameter Application (Advanced)**: For a 3-DOF spherical wrist (three revolute joints whose axes intersect at a single point), apply the DH convention to assign frames and derive the DH parameters. Explain why this specific configuration simplifies kinematics. Assume the first joint rotates about Z, the second about Y, and the third about Z again.
2.  **Inverse Kinematics Challenge**: Explain, with a concrete example (e.g., a 6-DOF industrial arm), why Inverse Kinematics often yields multiple solutions. How might a robot controller decide which solution to choose in a practical scenario (e.g., avoiding obstacles, minimizing joint travel)?
3.  **Jacobian for a Cylindrical Robot**: A cylindrical robot has a revolute joint for rotation about the Z-axis, followed by a prismatic joint for vertical extension along the Z-axis, and another prismatic joint for radial extension. Qualitatively describe how you would construct its Jacobian matrix to relate joint velocities (angular velocity for revolute, linear velocities for prismatic) to the end-effector's linear velocity components (`vx`, `vy`, `vz`).
4.  **Singularity Avoidance**: Describe a real-world task where encountering a robot singularity would be problematic. Suggest two strategies that could be employed in path planning or control to avoid such singular configurations.

##### Multiple Choice Questions

:::info
Which of the following statements is TRUE regarding Forward Kinematics (FK) and Inverse Kinematics (IK)?
- [ ] FK is generally more complex to compute than IK.
- [x] IK often has multiple solutions, while FK typically has one unique solution.
- [ ] FK calculates joint angles from end-effector pose, while IK calculates end-effector pose from joint angles.
- [ ] Both FK and IK are used to determine forces acting on robot joints.
:::

:::info
What do the four parameters (`a`, `α`, `d`, `θ`) in the Denavit-Hartenberg (DH) convention define?
- [ ] The material properties of each robot link.
- [ ] The forces and torques acting on each joint.
- [x] The geometric relationship between successive coordinate frames of a robot manipulator.
- [ ] The maximum speed and acceleration limits of a robot.
:::

:::info
Homogeneous Transformation Matrices (HTMs) are used to combine which two types of transformations into a single matrix?
- [ ] Scaling and Shearing
- [x] Rotation and Translation
- [ ] Reflection and Projection
- [ ] Velocity and Acceleration
:::

:::info
A robot is said to be in a **kinematic singularity** when:
- [ ] All its joints are locked in place.
- [ ] It has achieved its maximum operational speed.
- [x] Its Jacobian matrix loses rank, resulting in a loss of task-space degrees of freedom.
- [ ] It successfully grasps an object.
:::

:::info
The Jacobian matrix relates:
- [ ] End-effector forces to joint torques.
- [ ] Joint positions to end-effector positions.
- [x] Joint velocities to end-effector linear and angular velocities.
- [ ] Robot mass to gravity.
:::
