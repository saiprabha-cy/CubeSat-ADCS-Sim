function r_eci = orbit_propagator(t, alt_km, inc_deg)
% ORBIT_PROPAGATOR  Simple circular, inclined, two-body orbit position.
%
%   r_eci = orbit_propagator(t, alt_km, inc_deg)
%
%   t        : time (s) since ascending node crossing (RAAN = 0, theta0 = 0 assumed)
%   alt_km   : orbital altitude above Earth's surface (km), e.g. 500 for typical CubeSat LEO
%   inc_deg  : orbital inclination (deg)
%
%   Returns r_eci: 3x1 position vector in ECI frame, meters.
%
%   SIMPLIFICATIONS (documented, not hidden):
%     - Circular orbit only (no eccentricity)
%     - No J2 perturbation (real orbits precess; this one doesn't)
%     - RAAN = 0, argument of latitude = n*t (starts at ascending node)
%   This is sufficient for slowly-varying B-field lookups during a detumble
%   simulation (minutes to tens of minutes), where these simplifications
%   don't materially affect the result. Add J2/eccentricity later if the
%   project extends to multi-orbit, long-duration pointing scenarios.

    if nargin < 2, alt_km = 500; end
    if nargin < 3, inc_deg = 51.6; end   % ISS-like inclination, common CubeSat deployment

    mu = 3.986004418e14;   % Earth gravitational parameter, m^3/s^2
    R_E = 6378137;          % Earth mean equatorial radius, m

    R = R_E + alt_km * 1000;
    n = sqrt(mu / R^3);     % mean motion, rad/s
    theta = n * t;          % argument of latitude at time t

    inc = deg2rad(inc_deg);

    % Position in the orbital plane, then rotate into ECI by inclination
    % (rotation about the ECI X-axis, RAAN = 0 so ascending node is along X)
    r_orbit_plane = R * [cos(theta); sin(theta); 0];

    Rx = [1, 0, 0;
          0, cos(inc), -sin(inc);
          0, sin(inc),  cos(inc)];

    r_eci = Rx * r_orbit_plane;
end