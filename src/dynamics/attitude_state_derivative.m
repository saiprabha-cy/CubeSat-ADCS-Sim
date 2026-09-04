function state_dot = attitude_state_derivative(t, state, I, tau_total_fn)
% ATTITUDE_STATE_DERIVATIVE  Combined ODE right-hand side for ode45.
%
%   state_dot = attitude_state_derivative(t, state, I, tau_total_fn)
%
%   state        : 7x1 = [wx; wy; wz; q0; q1; q2; q3]
%   I            : 3x3 inertia tensor
%   tau_total_fn : function handle @(t, omega, q) -> [tx;ty;tz], the TOTAL
%                  torque at this instant. For the torque-free check this
%                  is just @(t,w,q) [0;0;0]. Later this will call into
%                  detumble_bdot.m / pid_attitude_control.m + disturbance
%                  models and sum the results.
%
%   ode45 requires a single state vector and a single derivative vector,
%   which is why omega (dynamics) and q (kinematics) are packed together
%   here even though they're implemented in separate files -- this
%   function is the ONLY place that couples them for integration purposes.
%
%   NOTE ON NORMALIZATION: ode45 will NOT keep q on the unit sphere by
%   itself -- numerical integration error accumulates. This function does
%   NOT renormalize (renormalizing mid-derivative-evaluation is not
%   correct ODE practice). Instead, the calling script renormalizes q
%   AFTER ode45 returns the full solution, and for long simulations should
%   re-integrate in short chunks with renormalization between chunks.
%   See sim/main_torque_free_check.m for how this is handled.

    omega = state(1:3);
    q     = state(4:7);

    tau_total = tau_total_fn(t, omega, q);

    omega_dot = rigid_body_dynamics(omega, I, tau_total);
    q_dot     = kinematics_quaternion(q, omega);

    state_dot = [omega_dot; q_dot];
end