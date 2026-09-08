%% MAIN_POINTING.M
% Build-order step 4: starting from a near-detumbled state, use the PD
% quaternion controller + reaction wheels to slew to and hold a target
% attitude. Confirm: (1) attitude error converges to ~0, (2) wheel
% momentum stays under h_max (i.e. the maneuver doesn't require
% desaturation partway through -- if it does, that's a real finding to
% report, not a failure to hide).

clear; clc; close all;
addpath('../src/dynamics');
addpath('../src/actuators');
addpath('../src/control');
addpath('../src/utils');

%% Setup
I_body = inertia_properties();   % default 3U CubeSat

% Initial condition: residual tumble left over after detumbling (roughly
% matching the ~0.042 rad/s final state from main_detumble.m), not a
% perfect zero -- a pointing controller should be able to handle small
% residual rates, that's realistic post-detumble handoff conditions
omega0 = [0.02; -0.015; 0.025];   % rad/s
q0 = [1; 0; 0; 0];                 % start aligned with inertial frame
h_wheel0 = [0; 0; 0];               % wheels start unspun

state0 = [omega0; q0; h_wheel0];

% Target attitude: 45 deg rotation about the [1,1,1] axis -- deliberately
% NOT aligned with any single body axis, so all three reaction wheels are
% genuinely exercised together rather than testing one axis at a time
target_angle_deg = 45;
target_axis = [1;1;1] / norm([1;1;1]);
theta = deg2rad(target_angle_deg);
q_target = [cos(theta/2); target_axis * sin(theta/2)];

% Reaction wheel actuator limits (matching reaction_wheel_model.m defaults)
h_max = 0.01;      % N*m*s per axis
tau_max = 0.0006;  % N*m per axis

% Initial PD gains -- see note below on how these were chosen
Kp = 3e-3;
Kd = 1.2e-2;

control_fn = @(q, omega) pid_attitude_control(q, q_target, omega, Kp, Kd);
tau_ext_fn = @(t, q) [0;0;0];   % no disturbance torque in this first pointing test
                                  % (signature is @(t,q) even though unused here,
                                  % to match attitude_state_derivative_with_wheels.m's
                                  % interface -- see main_disturbance_rejection.m for
                                  % where q is actually used)

t_span = [0, 800];   % seconds -- start shorter than the detumble runs;
                      % pointing settling times are typically much faster
                      % than detumble since we're not waiting on orbital
                      % field geometry, just actuator torque authority

%% Integrate
opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);
[t, state] = ode45(@(t,s) attitude_state_derivative_with_wheels( ...
    t, s, I_body, control_fn, h_max, tau_max, tau_ext_fn), t_span, state0, opts);

omega_hist   = state(:, 1:3);
q_hist       = state(:, 4:7);
h_wheel_hist = state(:, 8:10);

n = length(t);
for k = 1:n
    q_hist(k,:) = quaternion_ops.quat_normalize(q_hist(k,:)');
end

%% Compute attitude error angle over time (for reporting/plotting)
err_angle_deg = zeros(n,1);
for k = 1:n
    q_err = quaternion_ops.quat_error(q_target, q_hist(k,:)');
    if q_err(1) < 0, q_err = -q_err; end
    err_angle_deg(k) = 2 * rad2deg(acos(min(max(q_err(1),-1),1)));
end

h_wheel_mag = vecnorm(h_wheel_hist, 2, 2);

%% Report
fprintf('--- Pointing maneuver results ---\n');
fprintf('Target rotation: %.1f deg about [1,1,1] axis\n', target_angle_deg);
fprintf('Initial attitude error: %.2f deg\n', err_angle_deg(1));
fprintf('Final attitude error:   %.4f deg\n', err_angle_deg(end));
fprintf('Max |h_wheel| reached:  %.5f N*m*s (limit = %.5f N*m*s, %.1f%% of capacity)\n', ...
    max(h_wheel_mag), h_max, 100*max(h_wheel_mag)/h_max);

settle_threshold_deg = 1.0;
idx = find(err_angle_deg < settle_threshold_deg, 1, 'first');
if isempty(idx)
    fprintf('Did NOT settle within %.0f%% threshold in %.0f s.\n', settle_threshold_deg, t_span(2));
else
    fprintf('Settled below %.1f deg error at t = %.1f s\n', settle_threshold_deg, t(idx));
end

%% Plots
figure('Name', 'Pointing Maneuver');

subplot(2,2,1);
plot(t, omega_hist, 'LineWidth', 1.3);
xlabel('Time (s)'); ylabel('\omega (rad/s)');
legend('\omega_x','\omega_y','\omega_z');
title('Angular velocity during maneuver');
grid on;

subplot(2,2,2);
plot(t, err_angle_deg, 'LineWidth', 1.5, 'Color', [0.85 0.1 0.1]);
xlabel('Time (s)'); ylabel('Attitude error (deg)');
title('Pointing error convergence');
grid on;

subplot(2,2,3);
plot(t, h_wheel_hist, 'LineWidth', 1.2);
hold on;
yline(h_max, '--k', 'h_{max}');
yline(-h_max, '--k');
xlabel('Time (s)'); ylabel('h_{wheel} (N m s)');
legend('h_x','h_y','h_z','Location','best');
title('Reaction wheel momentum (check vs saturation limit)');
grid on;

subplot(2,2,4);
plot(t, h_wheel_mag, 'LineWidth', 1.5);
hold on;
yline(h_max, '--k', 'h_{max}');
xlabel('Time (s)'); ylabel('|h_{wheel}| (N m s)');
title('Wheel momentum magnitude vs saturation limit');
grid on;

saveas(gcf, '../results/figures/pointing_result.png');
fprintf('\nFigure saved to results/figures/pointing_result.png\n');