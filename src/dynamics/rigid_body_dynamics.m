function omega_dot = rigid_body_dynamics(omega, I, tau_total)
% RIGID_BODY_DYNAMICS  Euler's rotational equations of motion.
%
%   omega_dot = rigid_body_dynamics(omega, I, tau_total)
%
%   omega     : [wx; wy; wz] angular velocity in body frame (rad/s)
%   I         : 3x3 inertia tensor (body frame, from inertia_properties.m)
%   tau_total : [tx; ty; tz] TOTAL torque in body frame (N*m) -- this must
%               already be the sum of control torque + disturbance torque
%               + any reaction-torque terms; this function does not know
%               or care about torque sources, only the net result.
%
%   Implements:
%       I*omega_dot + omega x (I*omega) = tau_total
%       =>  omega_dot = I^-1 * (tau_total - omega x (I*omega))
%
%   The cross term omega x (I*omega) is the gyroscopic coupling term --
%   it is why rotation about one axis affects the others even with zero
%   applied torque, and it's what makes this genuinely a 3-axis coupled
%   nonlinear system rather than three independent single-axis problems.

    omega = omega(:);
    tau_total = tau_total(:);

    gyroscopic_term = cross(omega, I * omega);
    omega_dot = I \ (tau_total - gyroscopic_term);   % I \ x  ==  inv(I)*x, numerically better
end