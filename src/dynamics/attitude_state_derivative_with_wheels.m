function state_dot = attitude_state_derivative_with_wheels(t, state, I_body, control_fn, h_max, tau_max, tau_ext_fn)
% ATTITUDE_STATE_DERIVATIVE_WITH_WHEELS  Combined ODE right-hand side for
% ode45, extended for reaction-wheel pointing control.
%
%   state_dot = attitude_state_derivative_with_wheels(t, state, I_body, control_fn, h_max, tau_max, tau_ext_fn)
%
%   state      : 10x1 = [wx;wy;wz; q0;q1;q2;q3; hx;hy;hz]
%                (omega, quaternion, wheel momentum -- wheel momentum is
%                now a genuine dynamic state, unlike the memoryless
%                magnetorquer case)
%   I_body     : 3x3 body inertia tensor
%   control_fn : function handle @(q, omega) -> tau_cmd (3x1 DESIRED torque),
%                e.g. @(q,w) pid_attitude_control(q, q_target, w, Kp, Kd)
%                with q_target/Kp/Kd already fixed by closure
%   h_max      : scalar, wheel momentum storage limit (N*m*s), passed to
%                reaction_wheel_model.m
%   tau_max    : scalar, wheel motor torque limit (N*m), passed to
%                reaction_wheel_model.m
%   tau_ext_fn : function handle @(t, q) -> 3x1 EXTERNAL disturbance torque
%                (NOT including the wheel reaction, which is handled
%                separately). Pass @(t,q) [0;0;0] for the undisturbed case.
%                Takes q as well as t because real disturbance torques
%                (e.g. gravity-gradient) depend on current attitude, not
%                just elapsed time -- an @(t)-only interface could not
%                express that dependency.
%
%   Pipeline each evaluation:
%     1. control_fn computes the DESIRED torque from current attitude error
%     2. reaction_wheel_model.m saturates it into an ACHIEVABLE wheel
%        momentum rate (h_wheel_dot) and the resulting actual body torque
%     3. rigid_body_dynamics_with_wheels.m computes omega_dot from the
%        full coupled equation (external torque + gyroscopic coupling
%        with wheel momentum + wheel reaction)
%     4. kinematics_quaternion.m computes q_dot as before
%     5. h_wheel_dot is itself the 10th-13th... i.e. 8th-10th state's
%        derivative -- the wheel momentum state is integrated directly
%        by ode45 alongside omega and q, since it has genuine memory.

    omega   = state(1:3);
    q       = state(4:7);
    h_wheel = state(8:10);

    tau_cmd = control_fn(q, omega);

    [~, h_wheel_dot, ~] = reaction_wheel_model(tau_cmd, h_wheel, h_max, tau_max);

    tau_ext = tau_ext_fn(t, q);

    omega_dot = rigid_body_dynamics_with_wheels(omega, I_body, h_wheel, h_wheel_dot, tau_ext);
    q_dot     = kinematics_quaternion(q, omega);

    state_dot = [omega_dot; q_dot; h_wheel_dot];
end