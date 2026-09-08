function tau_gg = disturbance_torques(r_eci, q, I_body, mu)
% DISTURBANCE_TORQUES  Gravity-gradient disturbance torque.
%
%   tau_gg = disturbance_torques(r_eci, q, I_body, mu)
%
%   r_eci  : 3x1 satellite position, ECI frame, meters (from orbit_propagator.m)
%   q      : 4x1 current attitude quaternion (inertial->body)
%   I_body : 3x3 body inertia tensor
%   mu     : Earth gravitational parameter, m^3/s^2 (default 3.986004418e14)
%
%   Returns tau_gg: 3x1 gravity-gradient torque, body frame (N*m).
%
%   Physics: a non-spherical mass distribution (I not a scalar multiple of
%   identity) experiences a torque in a gravity field because different
%   parts of the body are at slightly different distances/directions from
%   Earth's center, so gravity pulls unevenly across the body. This is the
%   DOMINANT disturbance torque for a LEO CubeSat (larger than aerodynamic
%   or solar radiation pressure torque at typical CubeSat altitudes) --
%   which is why this is the one implemented here; aero/SRP are noted as
%   future extensions in the folder structure but deliberately not built
%   yet (only the dominant term is needed to demonstrate steady-state
%   error under P-only/PD-only control, which is this stage's purpose).
%
%   Formula (standard result, e.g. Wertz "Spacecraft Attitude Determination
%   and Control"):
%       tau_gg = (3*mu / R^3) * (o3 x (I_body * o3))
%   where o3 is the NADIR-pointing unit vector (satellite toward Earth
%   center) expressed in BODY frame, and R = |r_eci|.
%
%   Sanity check built into the physics: if the body were spherically
%   symmetric (I_body = c*eye(3) for scalar c), o3 x (I*o3) = c*(o3 x o3) = 0
%   -- zero gravity-gradient torque for a symmetric mass distribution,
%   exactly as expected physically.

    if nargin < 4, mu = 3.986004418e14; end

    R = norm(r_eci);
    nadir_eci = -r_eci(:) / R;                          % Earth-center-pointing unit vector, ECI
    o3 = quaternion_ops.quat_rotate(q, nadir_eci);       % same vector, expressed in body frame

    tau_gg = (3 * mu / R^3) * cross(o3, I_body * o3);
end