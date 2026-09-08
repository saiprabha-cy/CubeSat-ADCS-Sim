function state_dot = attitude_state_derivative_pid(t, state, I_body, control_fn, h_max, tau_max, tau_ext_fn)
% ATTITUDE_STATE_DERIVATIVE_PID  Combined ODE right-hand side for ode45,
% extended to include the integral-of-error state for full PID control.
%
%   state_dot = attitude_state_derivative_pid(t, state, I_body, control_fn, h_max, tau_max, tau_ext_fn)
%
%   state      : 13x1 = [wx;wy;wz; q0;q1;q2;q3; hx;hy;hz; eix;eiy;eiz]
%                (omega, quaternion, wheel momentum, integral-of-error --
%                FOUR genuine dynamic states now, each with its own memory)
%   control_fn : function handle @(q, omega, e_int) -> [tau_cmd, e_int_dot],
%                e.g. @(q,w,ei) pid_attitude_control_full(q, q_target, w, ei, Kp, Ki, Kd, e_int_max)
%                Note the different signature from the PD-only version
%                (attitude_state_derivative_with_wheels.m's control_fn only
%                returns tau_cmd) -- this function is DELIBERATELY separate
%                rather than overloading that one, since the PD-only path
%                is already validated (0.0000 deg pointing result) and
%                should not be touched.
%   h_max, tau_max, tau_ext_fn : same as attitude_state_derivative_with_wheels.m

    omega   = state(1:3);
    q       = state(4:7);
    h_wheel = state(8:10);
    e_int   = state(11:13);

    [tau_cmd, e_int_dot] = control_fn(q, omega, e_int);

    [~, h_wheel_dot, ~] = reaction_wheel_model(tau_cmd, h_wheel, h_max, tau_max);

    tau_ext = tau_ext_fn(t, q);

    omega_dot = rigid_body_dynamics_with_wheels(omega, I_body, h_wheel, h_wheel_dot, tau_ext);
    q_dot     = kinematics_quaternion(q, omega);

    state_dot = [omega_dot; q_dot; h_wheel_dot; e_int_dot];
end