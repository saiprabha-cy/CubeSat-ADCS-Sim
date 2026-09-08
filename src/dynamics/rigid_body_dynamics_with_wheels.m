function omega_dot = rigid_body_dynamics_with_wheels(omega, I_body, h_wheel, h_wheel_dot, tau_ext)
% RIGID_BODY_DYNAMICS_WITH_WHEELS  Euler's equations extended for a body
% carrying momentum-exchange reaction wheels.
%
%   omega_dot = rigid_body_dynamics_with_wheels(omega, I_body, h_wheel, h_wheel_dot, tau_ext)
%
%   omega      : [wx;wy;wz] body angular velocity (rad/s)
%   I_body     : 3x3 inertia tensor of the BODY excluding wheel spin
%                contribution (standard "reduced body inertia" formulation)
%   h_wheel    : [hx;hy;hz] current wheel angular momentum, body frame (N*m*s)
%   h_wheel_dot: [hx_dot;hy_dot;hz_dot] commanded rate of change of wheel
%                momentum = the actual motor torque being applied to the
%                wheels (N*m), AFTER actuator saturation (see reaction_wheel_model.m)
%   tau_ext    : [tx;ty;tz] EXTERNAL torque only (disturbance torques etc.)
%                -- does NOT include the wheel reaction torque, that is
%                already accounted for via the h_wheel terms below
%
%   Implements the standard spacecraft-with-wheels equation:
%       I_body*omega_dot = tau_ext - omega x (I_body*omega + h_wheel) - h_wheel_dot
%
%   Physical reading: total system momentum is (I_body*omega + h_wheel).
%   The cross term couples wheel momentum into the SAME gyroscopic coupling
%   that affected the wheel-free case -- a spinning wheel changes how the
%   body responds to its own rotation, not just adds a torque. The
%   -h_wheel_dot term is the direct reaction: spinning the wheel up in one
%   direction pushes the body the opposite way, by construction (Newton's
%   third law / conservation of total angular momentum absent tau_ext).
%
%   This function is DELIBERATELY separate from rigid_body_dynamics.m
%   (used for the verified torque-free and detumble cases) rather than
%   modifying it -- that file is already validated against energy/momentum
%   conservation and should not be touched now that a controller depends on it.

    omega = omega(:);
    h_wheel = h_wheel(:);
    h_wheel_dot = h_wheel_dot(:);
    tau_ext = tau_ext(:);

    total_momentum = I_body * omega + h_wheel;
    gyroscopic_term = cross(omega, total_momentum);

    omega_dot = I_body \ (tau_ext - gyroscopic_term - h_wheel_dot);
end