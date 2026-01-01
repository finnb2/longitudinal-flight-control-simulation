"""
2D Longitudinal Aircraft Dynamics and Flight Control Simulation
Author: Finn Benjamin
Description: Models pitch dynamics and closed-loop AoA control
"""

import matplotlib.pyplot as plt
import math
print("Flight Simulator Started")

#parameters
mass = 5.0
gravity = 9.81
rho = 1.225
S = 0.8
CL0 = 0.2
CLa = 5.0 # per radian
CD = 0.05
I = 1.0 
alpha_target = 0.07
thrust = 20.0
dt = 0.02
T_total = 5.0
steps = int(T_total/dt)
theta = 0.05
theta_dot = 0
Ka = 1.5
Kq = 0.8
Ke = 0.75
Kp = 0.5
use_controller = True
t_disturb = 1.0
disturb_mag = 0.05

#initial conditions
x = 0.0
y = 0.0
vx = 15.0
vy = 0.0
x_list = []
y_list = []
vy_list = []
time_list = []
alpha_list = []
theta_list = []
theta_dot_list = []
elevator_list = []

alpha_trim = mass * gravity / (0.5*rho*vx**2*S*CLa)
theta = alpha_trim
alpha_target = alpha_trim

for t in range(steps):
    if abs(t*dt - t_disturb) < dt:
        theta_dot += disturb_mag
    gamma = math.atan2(vy,vx)
    alpha = theta - gamma
    if use_controller:
        elevator = Kp * (alpha_target-alpha)
    else:
        elevator = 0
    elevator = max(min(elevator, 0.3), -0.3)
    Moment = Ke*elevator - Ka*alpha - Kq*theta_dot
    theta_ddot = Moment/I
    theta_dot += theta_ddot*dt
    theta += theta_dot * dt
    V = (vx**2 + vy**2)**0.5
    CL = CL0 + CLa*alpha
    Lift = 0.5*rho*V**2*S*CL
    Drag = 0.5*rho*V**2*S*CD
    ax = (thrust - Drag)/mass
    ay = (Lift - mass*gravity)/mass
    vx += ax*dt
    vy += ay*dt
    x += vx*dt
    y += vy*dt
    x_list.append(x)
    y_list.append(y)
    time_list.append(t*dt)
    alpha_list.append(alpha)
    theta_list.append(theta)
    theta_dot_list.append(theta_dot)
    vy_list.append(vy)
    elevator_list.append(elevator)

overshoot = max(0,max(alpha_list) - alpha_target)
tolerance = 0.02 * alpha_target
settling_time = None
for i in range(len(alpha_list)):
        if abs(alpha_list[i] - alpha_target) < tolerance:
            settling_time = time_list[i]
            break

'plt.plot(x_list, y_list)'
'plt.xlabel("forward distance (m)")'
'plt.ylabel("altitude(m)")'
'plt.title("2D Flight Simulation")'
'plt.grid(True)'
'plt.show()'

'plt.plot(time_list, alpha_list, label="a")'
'plt.axhline(alpha_target, linestyle="--", label = "a target")'
'plt.xlabel("time (s)")'
'plt.ylabel("Angle of Attack (rad)")'
'plt.title("Angle of Attack vs Time")'
'plt.grid(True)'
'plt.legend()'
'plt.show()'

'plt.figure()'
'plt.plot(time_list, theta_list)'
'plt.xlabel("Time (s)")'
'plt.ylabel("Pitch Angle θ (rad)")'
'plt.title("Pitch Angle vs Time")'
'plt.grid(True)'
'plt.show()'

'plt.figure()'
'plt.plot(time_list, y_list)'
'plt.xlabel("Time (s)")'
'plt.ylabel("Altitude (m)")'
'plt.title("Altitude vs Time")'
'plt.grid(True)'
'plt.show()'

'plt.figure()'
'plt.plot(time_list, elevator_list)'
'plt.xlabel("Time (s)")'
'plt.ylabel("Elevator Deflection")'
'plt.title("Elevator Input vs Time")'
'plt.grid(True)'
'plt.show()'

plt.figure(figsize = (8,6))

plt.subplot(3,1,1)
plt.plot(time_list,alpha_list)
plt.axhline(alpha_target,linestyle="--")
plt.ylabel("a (rad)")
plt.grid(True)

plt.subplot(3,1,2)
plt.plot(time_list,theta_list)
plt.ylabel("theta (rad)")
plt.grid(True)

plt.subplot(3,1,3)
plt.plot(time_list,elevator_list)
plt.xlabel("Time (s)")
plt.ylabel("Elevator")
plt.grid(True)

plt.suptitle("Longitudinal Response to Pitch Disturbance")
plt.tight_layout()
plt.show()

print(f'tolerance = {tolerance}')
print(f'overshoot = {overshoot}')