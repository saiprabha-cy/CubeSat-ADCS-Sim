function tau_cmd = pid_attitude_control(q, q_target, omega, Kp, Kd)
% PID_ATTITUDE_CONTROL  Quaternion-based attitude pointing controller.
%
%   tau_cmd = pid_attitude_control(q, q_target, omega, Kp, Kd)
%
%   q        : 4x1 current attitude quaternion
%   q_target : 4x1 target attitude quaternion (regulation to a fixed target;
%              for a tracking trajectory, pass the time-varying target here)
%   omega    : 3x1 current angular velocity, body frame (rad/s)
%   Kp, Kd   : scalar (or 3x1 per-axis) proportional and derivative gains
%
%   Returns tau_cmd: 3x1 COMMANDED torque (N*m), body frame -- feed this
%   into reaction_wheel_model.m for actuator saturation before it's applied
%   to the dynamics.
%
%   Control law:
%       tau_cmd = -Kp * sign(q_err0) * q_err_vec  -  Kd * omega
%
%   where q_err = quaternion_ops.quat_error(q_target, q), q_err0 is its
%   scalar part and q_err_vec its vector part. The vector part of a small
%   attitude error is ~proportional to the physical error angle, which is
%   why it can be used directly as a proportional-control input without
%   converting to Euler angles.
%
%   THE sign(q_err0) TERM -- do not remove this, it fixes a real bug class:
%   a quaternion and its negative (-q) represent the SAME physical
%   attitude, but q_err and -q_err give OPPOSITE-signed control torques.
%   Without this correction, the controller can occasionally command the
%   "long way around" (e.g. rotate -350 deg instead of +10 deg) to reach
%   the same physical target -- this is the well-documented "unwinding"
%   phenomenon in quaternion attitude control. Flipping q_err when its
%   scalar part is negative guarantees shortest-path convergence.
%
%   NAMED "PID" TO MATCH THE PROJECT'S FOLDER STRUCTURE, BUT RUN AS PD
%   FOR THIS FIRST POINTING TEST: no integral term yet. A pure PD law is
%   standard for eigenaxis regulation to a fixed target with no persistent
%   disturbance torque. The integral term is deliberately deferred to the
%   disturbance-rejection phase (main_disturbance_rejection.m), where a
%   constant gravity-gradient-type torque actually creates a steady-state
%   error that P+D alone cannot fully null -- that is the correct,
%   motivated place to introduce Ki and its associated anti-windup logic,
%   not here where it would just be an untested extra tuning knob.

    q_err = quaternion_ops.quat_error(q_target, q);

    % Shortest-path fix: flip sign if scalar part is negative
    if q_err(1) < 0
        q_err = -q_err;
    end

    err_vec = q_err(2:4);

    tau_cmd = -Kp(:) .* err_vec(:) - Kd(:) .* omega(:);
end