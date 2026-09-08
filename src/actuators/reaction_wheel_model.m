function [tau_body, h_wheel_dot, tau_actual] = reaction_wheel_model(tau_cmd_body, h_wheel, h_max, tau_max)
% REACTION_WHEEL_MODEL  Reaction wheel actuator hardware model.
%
%   [tau_body, h_wheel_dot, tau_actual] = reaction_wheel_model(tau_cmd_body, h_wheel, h_max, tau_max)
%
%   tau_cmd_body : 3x1 DESIRED torque ON THE BODY, as output by the control
%             law (e.g. pid_attitude_control.m) -- one (conceptual) wheel
%             per axis; a real 3-wheel orthogonal assembly is being
%             approximated here, not modeled as an arbitrary wheel-to-body
%             mapping matrix. Document this simplification if you extend
%             to a 4-wheel pyramid config.
%   h_wheel : 3x1 CURRENT wheel angular momentum, body frame (N*m*s)
%   h_max   : scalar, momentum saturation limit per wheel (N*m*s).
%             Representative small-CubeSat wheel: ~0.005 to 0.01 N*m*s.
%   tau_max : scalar, max torque the wheel motor can produce (N*m).
%             Representative small-CubeSat wheel: ~0.0005 to 0.001 N*m.
%
%   Returns:
%     tau_body    : 3x1 torque actually applied TO THE SATELLITE BODY (N*m),
%                   after actuator saturation -- approx equal to
%                   tau_cmd_body when unsaturated (see fix note below).
%     h_wheel_dot : 3x1 rate of change of wheel momentum = tau_actual
%                   (integrate this in the state vector to track wheel
%                   spin-up over time and watch for saturation approach).
%     tau_actual  : 3x1 torque actually applied to the WHEEL after both
%                   torque-limit and momentum-limit saturation.
%
%   *** SIGN CONVENTION FIX (documented, not silently patched) ***
%   By Newton's third law, torque on the body = -(torque on the wheel).
%   So to DELIVER a desired body torque tau_cmd_body, the motor must be
%   commanded to the OPPOSITE value: tau_wheel_cmd = -tau_cmd_body.
%   An earlier version of this file skipped that negation and clamped
%   tau_cmd_body directly as if it were the wheel-side command, which
%   silently inverted every control action (the controller would compute
%   a correct restoring torque, and the actuator would then apply its
%   negative to the body) -- this produced exactly the divergence/runaway
%   behavior seen in the first main_pointing.m run (wheels saturating fast
%   while attitude error grew instead of shrinking). Fixed here by
%   explicitly negating before saturation.
%
%   Saturation logic (two independent physical limits, both enforced):
%     1. Torque limit: the motor can only produce so much torque regardless
%        of momentum state -- straightforward per-axis clamp.
%     2. Momentum limit: once a wheel is at h_max, it CANNOT be commanded
%        to spin faster in that direction -- physically it's already at
%        max RPM. If already saturated and the command would push further
%        into saturation, that axis's torque is zeroed (the wheel simply
%        cannot comply -- in a real mission this is exactly the trigger
%        for a magnetic desaturation maneuver, not implemented here but
%        worth noting in your results doc as a known next step).

    if nargin < 3, h_max = 0.01; end     % N*m*s
    if nargin < 4, tau_max = 0.0006; end % N*m

    tau_cmd_body = tau_cmd_body(:);
    h_wheel = h_wheel(:);

    tau_wheel_cmd = -tau_cmd_body;   % <-- the fix: negate before treating as wheel command

    % 1) torque-limit saturation
    tau_actual = zeros(3,1);
    for i = 1:3
        tau_actual(i) = max(min(tau_wheel_cmd(i), tau_max), -tau_max);
    end

    % 2) momentum-limit saturation: zero torque on any axis already
    %    saturated in the direction the command is pushing
    for i = 1:3
        if (h_wheel(i) >= h_max && tau_actual(i) > 0) || ...
           (h_wheel(i) <= -h_max && tau_actual(i) < 0)
            tau_actual(i) = 0;
        end
    end

    h_wheel_dot = tau_actual;
    tau_body = -tau_actual;   % Newton's third law: reaction on the body
end