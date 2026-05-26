import control
import matplotlib.pyplot as plt
# Define the system's transfer function (example: first-order system)
num = [1]
den = [1, 2]
sys = control.TransferFunction(num, den)
# Design a PID controller
kp = 1.0
ki = 0.5
kd = 0.1
pid_controller = control.tf([kd, kp, ki], [1, 0])
# Create the closed-loop system
closed_loop_sys = control.feedback(sys, pid_controller)
# Simulate the step response
t, y = control.step_response(closed_loop_sys)
plt.plot(t, y)
plt.xlabel('Time')
plt.ylabel('Output')
plt.title('Step Response of Closed-Loop System with PID Controller')
plt.grid(True)
plt.show()