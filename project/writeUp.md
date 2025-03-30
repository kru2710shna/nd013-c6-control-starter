# Control and Trajectory Tracking for Autonomous Vehicle

## Proportional-Integral-Derivative (PID)

# File project/pid_controller.cpp and pid_conroller.h

### Class Summary (PID)
The PID class implements a PID controller to minimize the cross-track error (CTE) in self-driving car applications. It computes control outputs based on proportional, integral, and derivative errors while enforcing limits.

### Member Variables:
p_error, i_error, d_error:
Store the proportional, integral, and derivative errors.

Kp, Ki, Kd:
The PID gain coefficients for proportional, integral, and derivative control.

output_lim_max, output_lim_min:
Define the upper and lower bounds for the output control value.

delta_time:
Time interval between updates; used for integral and derivative calculations.

### Functions:
PID() (Constructor)
Initializes the PID object.

~PID() (Destructor)
Cleans up PID resources (default behavior here).

Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min)
Sets PID gains, output limits, and resets errors and delta time.

UpdateError(double cte, bool debugMode)
Updates p_error, i_error, and d_error using the new cross-track error (cte) and time step. Optional debug logging.

TotalError()
Computes the total PID output using the error terms and clamps it within [output_lim_min, output_lim_max].

UpdateDeltaTime(double new_delta_time)
Updates the time difference between controller updates and returns it.

## File Project/main.cpp

High-Level Summary

The code implements a self-driving car path planning and control loop using Carla simulator, where:

A Behavior Planner determines high-level maneuvers (e.g., STOPPED).
A Motion Planner generates candidate paths (spirals) based on the vehicle's goal.
Two PID controllers (for steering and throttle) are used to minimize the control error.
Control values are computed and sent via WebSocket (uWS) to actuate the vehicle.
Performance data is logged to text files for visualization or analysis.

## Results 

