# LRT End Effector Kinematics

A C++ library for forward and inverse kinematics of multibody robotic systems, built on top of [Pinocchio](https://github.com/stack-of-tasks/pinocchio) and the [OCS2](https://github.com/leggedrobotics/ocs2) framework.

The library supports multiple end effectors, position-only and full-pose kinematic tasks, differential kinematics, joint limit validation, and multiple inverse kinematics solvers.

## Features

* Forward kinematics for multiple end effectors
* Inverse kinematics for position and pose targets
* 3-DoF end effectors with position-only tasks
* 6-DoF end effectors with position and orientation tasks
* Mixed 3-DoF and 6-DoF end-effector configurations
* Joint velocity calculation from desired end-effector velocities
* End-effector velocity calculation from joint velocities
* Stacked end-effector Jacobian calculation
* Joint position and velocity limit validation
* Automatic detection of independent kinematic groups
* Support for redundant and non-redundant kinematic tasks
* Damped solving near singular configurations
* Multiple inverse kinematics solvers
* ROS 2 visualization demo using the R6Bot model
* GoogleTest-based tests and Google Benchmark-based solver benchmarks

## Supported Solvers

The library currently provides the following inverse kinematics solvers:

| Solver            | Description                                                            |
| ----------------- | ---------------------------------------------------------------------- |
| `NewtonRaphson`   | Newton-Raphson solver using an analytical task Jacobian                |
| `NewtonRaphsonAD` | Newton-Raphson solver using automatic differentiation                  |
| `QuIK`            | Higher-order inverse kinematics solver using automatic differentiation |

The solver is selected when constructing `MultiEndEffectorKinematics`.

## Dependencies

The core library uses:

* C++17
* Eigen3
* Pinocchio
* OCS2
* `ocs2_pinocchio_interface`
* `ocs2_robotic_tools`
* Boost
* ROS 2 `ament_cmake`

The R6Bot visualization demo additionally requires:

* `rclcpp`
* `sensor_msgs`
* `visualization_msgs`
* `ament_index_cpp`
* `robot_state_publisher`
* `rviz2`
* `xacro`
* `launch`
* `launch_ros`

## Building

Place the package inside the `src` directory of a ROS 2 workspace:

```text
workspace/
└── src/
    └── LRT_end_effector_kinematics/
```

Build the package from the workspace root:

```bash
cd ~/workspace
colcon build --packages-select multi_end_effector_kinematics
source install/setup.bash
```

## Basic Usage

The main entry point of the library is the `MultiEndEffectorKinematics` class.

A typical inverse kinematics workflow consists of:

1. Defining the base frame and end effectors.
2. Configuring the inverse kinematics solver.
3. Creating a `MultiEndEffectorKinematics` instance from a URDF model.
4. Defining the desired end-effector target.
5. Calling `calculateJointPositions()`.
6. Checking the returned `ReturnStatus`.

A complete executable example is available in:

```text
examples/BasicInverseKinematics.cpp
```

## Multi-End-Effector Handling

The library automatically determines which end effectors must be solved together.

Two end effectors belong to the same kinematic group when their kinematic chains share at least one active velocity DoF.

For example:

```text
End effector A ── joints 1, 2, 3 ──┐
                                   ├── Kinematic group 1
End effector B ── joints 2, 4 ─────┘

End effector C ── joints 5, 6 ───────── Kinematic group 2
```

In this case, end effectors A and B must be solved together because they share joint DoF 2. End effector C is independent and can be handled as a separate kinematic group.

During initialization, the library:

1. Determines which velocity-space DoFs influence each end effector.
2. Detects shared DoFs between end effectors.
3. Builds connected kinematic groups.
4. Assigns the corresponding task rows and Jacobian columns to each group.

This allows independent parts of a multi-end-effector problem to be solved separately instead of treating the complete robot as one coupled task.

## API Overview

The most important functions provided by `MultiEndEffectorKinematics` are:

| Function                           | Description                                                                    |
| ---------------------------------- | ------------------------------------------------------------------------------ |
| `calculateJointDeltas()`           | Performs a single inverse kinematics iteration and calculates joint increments |
| `calculateJointPositions()`        | Solves an inverse kinematics position or pose task                             |
| `calculateJointVelocities()`       | Calculates joint velocities from desired end-effector velocities               |
| `calculateEndEffectorPoses()`      | Calculates end-effector positions and poses using forward kinematics           |
| `calculateEndEffectorVelocities()` | Calculates end-effector velocities from joint velocities                       |
| `getJacobian()`                    | Returns the stacked Jacobian for all configured end effectors                  |
| `checkPositionBounds()`            | Checks joint positions against model limits                                    |
| `checkVelocityBounds()`            | Checks joint velocities against model limits                                   |
| `getPinocchioModel()`              | Provides access to the underlying Pinocchio model                              |

## Return Status

Kinematic operations return a `ReturnStatus` structure. The returned status should always be checked before using the calculated solution.

Possible task flags include:

| Flag                             | Meaning                                                                  |
| -------------------------------- | ------------------------------------------------------------------------ |
| `FINISHED`                       | The task completed successfully                                          |
| `IN_PROGRESS`                    | The solver did not reach a final state yet                               |
| `SOLVER_ERROR`                   | The selected solver failed                                               |
| `CURRENT_POSITION_OUT_OF_BOUNDS` | Initial joint positions violate model limits                             |
| `CURRENT_VELOCITY_OUT_OF_BOUNDS` | Initial joint velocities violate model limits                            |
| `NEW_POSITION_OUT_OF_BOUNDS`     | Calculated joint positions violate model limits                          |
| `NEW_VELOCITY_OUT_OF_BOUNDS`     | Calculated joint velocities violate model limits                         |
| `SMALL_STEP_SIZE`                | The solver step became too small before reaching the requested tolerance |

## R6Bot IK Visualization Demo

The package contains a ROS 2 demo showing iterative inverse kinematics on the R6Bot model in RViz.

After building and sourcing the workspace, run:

```bash
ros2 launch multi_end_effector_kinematics R6BotIkDemoNodeLaunch.py
```

The launch file starts:

* `robot_state_publisher`
* `R6BotIkDemoNode`
* RViz

The demo visualizes the configured target pose and the TCP trajectory while the inverse kinematics algorithm iteratively moves the robot toward the target.

The demo parameters can be modified in:

```text
launch/R6BotIkDemoNodeLaunch.py
```

The available configuration includes:

* solver name
* solver step coefficient
* initial joint positions
* target position
* target orientation in roll-pitch-yaw representation
* visualization iteration period
* maximum number of demo iterations

## Tests

Build the package and run its test suite with:

```bash
colcon test --packages-select multi_end_effector_kinematics
colcon test-result --verbose
```

The test suite covers the available inverse kinematics solvers and the `MultiEndEffectorKinematics` interface.

## Benchmarks

Solver benchmarks can be enabled with the `ACTIVE_BENCHMARKS` CMake option:

```bash
colcon build \
  --packages-select multi_end_effector_kinematics \
  --cmake-args -DACTIVE_BENCHMARKS=ON
```

The benchmark implementation is located in:

```text
benchmarks/SolversBenchmark.cpp
```

It can be used to compare the performance of the available inverse kinematics solvers on the provided robot models and target configurations.

## Project Structure

```text
LRT_end_effector_kinematics/
├── benchmarks/        # Solver benchmarks
├── examples/          # Minimal library usage examples
├── include/
│   └── multi_end_effector_kinematics/
│       ├── solvers/   # Public solver interfaces
│       ├── MultiEndEffectorKinematics.hpp
│       └── Settings.hpp
├── launch/            # ROS 2 demo launch files
├── src/
│   ├── solvers/       # Solver implementations
│   ├── MultiEndEffectorKinematics.cpp
│   └── R6BotIkDemoNode.cpp
├── test/              # Unit and integration tests with test robot models
├── CMakeLists.txt
├── package.xml
└── README.md
```

## License

This project is licensed under the GNU General Public License v3.0.

See the [LICENSE](LICENSE) file for details.
