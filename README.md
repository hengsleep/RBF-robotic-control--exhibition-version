# Adaptive RBF Neural Control for Multi-Joint Robotic Manipulators

## Overview

This repository implements an adaptive Radial Basis Function (RBF) neural network control scheme for the trajectory tracking of uncertain robotic manipulators. The architecture integrates a computed torque control framework with online RBF neural network approximation to compensate for unknown nonlinear dynamics, external disturbances, and modeling uncertainties.

## Architecture and Functional Description

The system utilizes a modular control architecture that separates trajectory tracking from dynamic compensation. The outer-loop controller computes the virtual acceleration command to guarantee the desired second-order error dynamics for tracking performance. The inner-loop controller realizes this command by generating the actual joint torque through nominal dynamic inversion coupled with an RBF neural network compensator. To ensure closed-loop stability and prevent parameter drift in the presence of persistent approximation errors, the neural network weights are updated online using a Lyapunov-derived adaptive law equipped with a $\sigma$-modification technique. This guarantees that all closed-loop signals, including the tracking error and weight estimation errors, remain uniformly ultimately bounded.

## Core Mathematical Formulations

The rigid robotic manipulator is governed by the standard Euler-Lagrange dynamic equation:
$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = 	au + d$$

The control law generating the joint torque is designed as:
$$ au = M_0(q)v + C_0(q,\dot{q})\dot{q} + G_0(q) - \hat{f}(x)$$

where the virtual control input is defined as $v = \ddot{q}_d - K_v \dot{e} - K_p e$, and $\hat{f}(x)$ represents the lumped uncertainty estimated by the RBF network:
$$\hat{f}(x) = \hat{W}^T h(x)$$

The weights are updated continuously via the $\sigma$-modified adaptive law:
$$\dot{\hat{W}} = \gamma h(X)X^T P B - \sigma \gamma \|X\| \hat{W}$$

## Core Module Paths and File Examples

**`/src/dynamics/`**

- `two_link_manipulator.m`: Contains the implementation of the nominal inertia matrix $M_0(q)$, Coriolis/centrifugal matrix $C_0(q,\dot{q})$, and gravity vector $G_0(q)$ for a two-link planar robotic manipulator.
- `nonlinear_servo.slx`: Simulink model representing a single-degree-of-freedom nonlinear servo system subjected to non-smooth friction disturbances.

**`/src/controllers/`**

- `computed_torque_control.m`: Implements the outer-loop trajectory tracking controller, calculating the desired virtual acceleration based on position and velocity errors.
- `rbf_compensator.m`: Initializes and executes the RBF neural network (utilizing Gaussian basis functions) to compute $\hat{f}(x)$ for inner-loop dynamic compensation.

**`/src/adaptation/`**

- `weight_update_law.m`: Contains the numerical solver for the $\sigma$-modified adaptive law, utilizing the tracking error state vector and the solution $P$ to the Lyapunov equation.

**`/simulations/`**

- `run_servo_sim.m`: Initialization script to evaluate the 1-DOF system under nominal (S=1), exact (S=2), and RBF-compensated (S=3) control modes.
- `run_two_link_sim.m`: Main script executing the two-link robotic manipulator simulation, introducing a 20% parameter mismatch and nonlinear dynamic coupling to validate the robustness of the RBF adaptive controller.# Adaptive RBF Neural Control for Multi-Joint Robotic Manipulators

## Overview

This repository implements an adaptive Radial Basis Function (RBF) neural network control scheme for the trajectory tracking of uncertain robotic manipulators. The architecture integrates a computed torque control framework with online RBF neural network approximation to compensate for unknown nonlinear dynamics, external disturbances, and modeling uncertainties.

## Architecture and Functional Description

The system utilizes a modular control architecture that separates trajectory tracking from dynamic compensation. The outer-loop controller computes the virtual acceleration command to guarantee the desired second-order error dynamics for tracking performance. The inner-loop controller realizes this command by generating the actual joint torque through nominal dynamic inversion coupled with an RBF neural network compensator. To ensure closed-loop stability and prevent parameter drift in the presence of persistent approximation errors, the neural network weights are updated online using a Lyapunov-derived adaptive law equipped with a $\sigma$-modification technique. This guarantees that all closed-loop signals, including the tracking error and weight estimation errors, remain uniformly ultimately bounded.

## Core Mathematical Formulations

The rigid robotic manipulator is governed by the standard Euler-Lagrange dynamic equation:
$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = 	au + d$$

The control law generating the joint torque is designed as:
$$ au = M_0(q)v + C_0(q,\dot{q})\dot{q} + G_0(q) - \hat{f}(x)$$

where the virtual control input is defined as $v = \ddot{q}_d - K_v \dot{e} - K_p e$, and $\hat{f}(x)$ represents the lumped uncertainty estimated by the RBF network:
$$\hat{f}(x) = \hat{W}^T h(x)$$

The weights are updated continuously via the $\sigma$-modified adaptive law:
$$\dot{\hat{W}} = \gamma h(X)X^T P B - \sigma \gamma \|X\| \hat{W}$$

## Core Module Paths and File Examples

**`/src/dynamics/`**

- `two_link_manipulator.m`: Contains the implementation of the nominal inertia matrix $M_0(q)$, Coriolis/centrifugal matrix $C_0(q,\dot{q})$, and gravity vector $G_0(q)$ for a two-link planar robotic manipulator.
- `nonlinear_servo.slx`: Simulink model representing a single-degree-of-freedom nonlinear servo system subjected to non-smooth friction disturbances.

**`/src/controllers/`**

- `computed_torque_control.m`: Implements the outer-loop trajectory tracking controller, calculating the desired virtual acceleration based on position and velocity errors.
- `rbf_compensator.m`: Initializes and executes the RBF neural network (utilizing Gaussian basis functions) to compute $\hat{f}(x)$ for inner-loop dynamic compensation.

**`/src/adaptation/`**

- `weight_update_law.m`: Contains the numerical solver for the $\sigma$-modified adaptive law, utilizing the tracking error state vector and the solution $P$ to the Lyapunov equation.

**`/simulations/`**

- `run_servo_sim.m`: Initialization script to evaluate the 1-DOF system under nominal (S=1), exact (S=2), and RBF-compensated (S=3) control modes.
- `run_two_link_sim.m`: Main script executing the two-link robotic manipulator simulation, introducing a 20% parameter mismatch and nonlinear dynamic coupling to validate the robustness of the RBF adaptive controller.
