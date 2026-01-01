# longitudinal-flight-control-simulation
2D longitudinal aircraft dynamics simulation with closed-loop angle-of-attack control

# Aircraft Longitudinal Dynamics and Flight Control Simulation

This project models the 2D longitudinal dynamics of a fixed-wing aircraft and implements
a closed-loop elevator controller to regulate angle of attack about a trimmed flight condition.

## Features
- Point-mass longitudinal flight model
- Pitch dynamics with rotational inertia
- Linear aerodynamic model (CL = CL0 + CLa·α)
- Proportional elevator control for angle-of-attack regulation
- Actuator saturation modeling
- Time-history and trajectory visualization

## Model Assumptions
- 2D motion (no lateral dynamics)
- Small-angle approximations
- Linear lift curve (no stall modeling)
- Constant thrust

## Control Approach
A proportional controller commands elevator deflection based on angle-of-attack error.
Closed-loop response is analyzed following a pitch-rate disturbance and compared to
trimmed flight behavior.

## Results
The controller stabilizes the aircraft and regulates angle of attack with limited overshoot
and finite settling time. Actuator saturation introduces realistic transient behavior.

![Longitudinal Response](Figures/longitudinal_response.png)

## Future Improvements
- Nonlinear aerodynamics and stall modeling
- PI/PID control
- Actuator rate limiting
- Full 6-DOF dynamics
