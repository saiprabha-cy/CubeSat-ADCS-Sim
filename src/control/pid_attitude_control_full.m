function [tau_cmd, e_int_dot] = pid_attitude_control_full(q, q_target, omega, e_int, Kp, Ki, Kd, e_int_max)
% PID_ATTITUDE_CONTROL_FULL  Quaternion PID controller (adds integral term
% and anti-windup to the validated PD law in pid_attitude_control.m).
%
%   [tau_cmd, e_int_dot] = pid_attitude_control_full(q, q_target, omega, e_int, Kp, Ki, Kd, e_int_max)
%
%   q, q_target, omega : same as pid_attitude_control.m
%   e_int    : 3x1 CURRENT integral-of-error state (this is a genuine
%              dynamic state now, unlike pure P/D -- it must be integrated
%              by ode45 alongside omega/q/h_wheel, hence e_int_dot below)
%   Kp,Ki,Kd : proportional, integral, derivative gains
%   e_int_max: scalar, anti-windup clamp on |e_int| per axis
%
%   Returns:
%     tau_cmd   : 3x1 DESIRED body torque (N*m), same downstream usage
%                 as pid_attitude_control.m (feed to reaction_wheel_model.m)
%     e_int_dot : 3x1 rate of change of the integral state -- this is
%                 simply the (shortest-path-corrected) error vector itself,
%                 EXCEPT zeroed on any axis where anti-windup is active
%                 (see below). Integrate this in the ODE state vector.
%
%   Control law:
%       tau_cmd = -Kp.*qe_vec - Ki.*e_int - Kd.*omega
%       e_int_dot = qe_vec   (with anti-windup clamp)
%
%   ANTI-WINDUP (same saturation-aware philosophy as reaction_wheel_model.m's
%   momentum-limit logic): if |e_int(i)| is already at e_int_max AND the
%   incoming error would push it further in that direction, e_int_dot(i) is
%   zeroed for this step -- this prevents the classic "integral windup"
%   failure mode where a controller that's been fighting a large error for
%   a while overshoots badly once the error finally starts to shrink,
%   because the accumulated integral term is still commanding hard in the
%   old direction. Without this, a PID controller recovering from the
%   45deg initial slew error (large error, long transient) would almost
%   certainly overshoot on the way to steady-state.

    qe = quaternion_ops.quat_error(q_target, q);
    qe0 = qe(1);
    qe_vec = qe(2:4);
    if qe0 < 0
        qe_vec = -qe_vec;
    end

    e_int = e_int(:);
    e_int_dot = qe_vec(:);

    % Anti-windup: zero the integrator rate on any axis already saturated
    % in the direction the error is pushing
    for i = 1:3
        if (e_int(i) >= e_int_max && e_int_dot(i) > 0) || ...
           (e_int(i) <= -e_int_max && e_int_dot(i) < 0)
            e_int_dot(i) = 0;
        end
    end

    tau_cmd = -Kp(:).*qe_vec(:) - Ki(:).*e_int - Kd(:).*omega(:);
end