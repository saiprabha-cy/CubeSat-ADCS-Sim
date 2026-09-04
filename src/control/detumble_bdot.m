function m_cmd = detumble_bdot(omega, B_body, k)
% DETUMBLE_BDOT  B-dot detumbling control law.
%
%   m_cmd = detumble_bdot(omega, B_body, k)
%
%   omega  : [wx;wy;wz] angular velocity, body frame (rad/s)
%   B_body : [Bx;By;Bz] local magnetic field in body frame (Tesla)
%   k      : positive scalar gain (tune empirically -- start ~1e4 to 1e5
%            for SI units with B in Tesla and m in A*m^2, then adjust
%            based on detumble time vs actuator saturation trade-off)
%
%   Returns m_cmd: 3x1 COMMANDED magnetic dipole moment (A*m^2), BEFORE
%   actuator saturation -- apply magnetorquer_model.m next to get the
%   physically achievable moment and resulting torque.
%
%   Law: m_cmd = -k * B_dot_body
%
%   B_dot_body is computed ANALYTICALLY as -omega x B_body rather than by
%   finite-differencing across timesteps. This is the correct approach for
%   an ode45-integrated system: the solver evaluates the RHS at internal
%   stage points that are not strictly sequential in time, so a
%   "previous timestep" finite difference is not well-defined mid-integration.
%   The analytic form follows directly from the transport theorem for a
%   field that is quasi-static in the inertial frame (see derivation notes) --
%   it is exact under that assumption, not an approximation of convenience.

    omega = omega(:);
    B_body = B_body(:);

    B_dot_body = -cross(omega, B_body);
    m_cmd = -k * B_dot_body;
end