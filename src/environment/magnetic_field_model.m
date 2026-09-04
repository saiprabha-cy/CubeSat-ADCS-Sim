function B_eci = magnetic_field_model(r_eci, tilt_deg)
% MAGNETIC_FIELD_MODEL  Tilted-dipole approximation of Earth's magnetic field.
%
%   B_eci = magnetic_field_model(r_eci, tilt_deg)
%
%   r_eci    : 3x1 position vector in ECI frame, meters (from orbit_propagator.m)
%   tilt_deg : dipole tilt from the ECI Z-axis, degrees. Earth's actual
%              geomagnetic dipole is tilted ~11.7 deg from the rotation
%              axis; pass 0 for a first-pass untilted-dipole simplification.
%
%   Returns B_eci: 3x1 magnetic field vector in ECI frame, Tesla.
%
%   Physics (centered dipole model, standard Cartesian form):
%       B(r) = (B0 * RE^3 / |r|^3) * [3*(m_hat . r_hat)*r_hat - m_hat]
%   where m_hat is the dipole axis unit vector and r_hat = r/|r|.
%
%   This is the same physics as the spherical-coordinate (Br, B_theta)
%   formulation used in the derivation notes -- this Cartesian form avoids
%   an extra coordinate conversion and is more convenient once r is already
%   in ECI Cartesian from the orbit propagator.
%
%   NOTE: this is a static (non-rotating) dipole for simplicity -- in
%   reality the dipole axis is fixed in ECI while Earth (and the magnetic
%   field measurement geometry relative to a ground point) rotates
%   underneath it. For a satellite's B-dot detumbling on the timescale of
%   minutes, treating the dipole as ECI-fixed is standard and adequate.

    if nargin < 2, tilt_deg = 11.7; end

    B0 = 3.12e-5;        % Tesla, mean value at the equator at Earth's surface
    R_E = 6378137;        % meters

    tilt = deg2rad(tilt_deg);
    m_hat = [sin(tilt); 0; cos(tilt)];   % dipole axis, tilted in the X-Z plane (longitude=0)

    r_mag = norm(r_eci);
    r_hat = r_eci(:) / r_mag;

    B_eci = (B0 * R_E^3 / r_mag^3) * (3 * dot(m_hat, r_hat) * r_hat - m_hat);
end