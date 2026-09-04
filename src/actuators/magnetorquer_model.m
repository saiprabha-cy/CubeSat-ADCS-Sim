function [tau_body, m_actual] = magnetorquer_model(m_cmd, B_body, m_max)
% MAGNETORQUER_MODEL  Magnetorquer actuator hardware model.
%
%   [tau_body, m_actual] = magnetorquer_model(m_cmd, B_body, m_max)
%
%   m_cmd  : 3x1 COMMANDED dipole moment from the control law (A*m^2)
%   B_body : 3x1 local magnetic field, body frame (Tesla)
%   m_max  : scalar, max achievable dipole moment PER AXIS (A*m^2).
%            Typical CubeSat magnetorquer: ~0.1 to 0.2 A*m^2 per axis.
%
%   Returns:
%     m_actual : 3x1 ACHIEVABLE dipole moment after per-axis saturation
%     tau_body : 3x1 resulting torque, body frame (N*m) = m_actual x B_body
%
%   Deliberately separate from detumble_bdot.m: the control law doesn't
%   know or care about hardware limits, and the actuator model doesn't know
%   or care what control law produced the command -- this mirrors real GNC
%   software structure (controller vs. actuator driver) and means this file
%   is directly reusable if you swap in a different control law later.
%
%   NOTE: magnetorquers can ONLY produce torque perpendicular to B_body
%   (torque = m x B is always perpendicular to B by construction of the
%   cross product) -- there is no way to saturate around that; it's a
%   fundamental physical limitation of magnetic actuation, not a bug.

    if nargin < 3, m_max = 0.2; end   % A*m^2, representative CubeSat magnetorquer

    m_cmd = m_cmd(:);
    m_actual = zeros(3,1);
    for i = 1:3
        m_actual(i) = max(min(m_cmd(i), m_max), -m_max);   % per-axis saturation
    end

    tau_body = cross(m_actual, B_body(:));
end