function I = inertia_properties(dims_m, mass_kg)
% INERTIA_PROPERTIES  Diagonal inertia tensor for a rectangular-box CubeSat.
%
%   I = inertia_properties(dims_m, mass_kg)
%
%   dims_m   : [a b c] box dimensions in meters, e.g. a 3U CubeSat is
%              roughly [0.1, 0.1, 0.34] (10cm x 10cm x 34cm incl. rails)
%   mass_kg  : total mass in kg, e.g. a 3U CubeSat is typically ~4 kg
%
%   Returns a 3x3 diagonal inertia matrix (principal axes assumed aligned
%   with the body frame -- true for a uniform box, a reasonable first-pass
%   approximation for a CubeSat with a roughly symmetric internal layout).
%
%   Formula (solid uniform box, standard rigid-body mechanics result):
%     Ixx = (1/12)*m*(b^2 + c^2)
%     Iyy = (1/12)*m*(a^2 + c^2)
%     Izz = (1/12)*m*(a^2 + b^2)
%
%   NOTE: this is intentionally a simplification. A real CubeSat's mass is
%   not uniformly distributed (battery pack, PCBs, wheels are concentrated
%   masses) -- the box model is the correct STARTING point for this project,
%   not the final word. If you extend this later, add a parallel-axis-theorem
%   correction per component with known offset from the geometric center.

    if nargin < 2
        % Default: a representative 3U CubeSat
        dims_m  = [0.1, 0.1, 0.34];
        mass_kg = 4.0;
        fprintf('inertia_properties: using default 3U CubeSat (dims=[0.1 0.1 0.34] m, mass=4.0 kg)\n');
    end

    a = dims_m(1); b = dims_m(2); c = dims_m(3);
    m = mass_kg;

    Ixx = (1/12) * m * (b^2 + c^2);
    Iyy = (1/12) * m * (a^2 + c^2);
    Izz = (1/12) * m * (a^2 + b^2);

    I = diag([Ixx, Iyy, Izz]);
end