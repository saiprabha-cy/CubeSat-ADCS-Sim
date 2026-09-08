%% MAIN_DISTURBANCE_REJECTION.M
% Build-order step 5: same pointing maneuver and SAME pure-PD controller as
% main_pointing.m, now with a real gravity-gradient disturbance torque
% active throughout. Purpose is to DEMONSTRATE the expected steady-state
% error that P+D alone cannot fully null against a persistent disturbance
% -- this is correct control-theory behavior, not a bug, and it is the
% deliberate, motivated setup for adding an integral term next
% (see pid_attitude_control.m's own docstring note on why Ki was deferred
% to exactly this stage).

clear; clc; close all;
addpath('../src/dynamics');
addpath('../src/actuators');
addpath('../src/control');
addpath('../src/utils');
addpath('../src/environment');

%% Setup -- identical to main_pointing.m except tau_ext_fn
I_body = inertia_properties();

omega0 = [0.02; -0.015; 0.025];
q0 = [1; 0; 0; 0];
h_wheel0 = [0; 0; 0];
state0 = [omega0; q0; h_wheel0];

target_angle_deg = 45;
target_axis = [1;1;1] / norm([1;1;1]);
theta = deg2rad(target_angle_deg);
q_target = [cos(theta/2); target_axis * sin(theta/2)];

h_max = 0.01;
tau_max = 0.0006;

Kp = 3e-3;   % SAME gains as main_pointing.m, deliberately unchanged --
Kd = 1.2e-2; % this run isolates the disturbance as the only new variable

control_fn = @(q, omega) pid_attitude_control(q, q_target, omega, Kp, Kd);

alt_km = 500;
inc_deg = 51.6;
tau_ext_fn = @(t, q) disturbance_torques(orbit_propagator(t, alt_km, inc_deg), q, I_body);

t_span = [0, 2000];   % longer than main_pointing.m's 800s -- need enough
                       % time to clearly see the settled STEADY-STATE
                       % error plateau, not just the initial transient

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

err_angle_deg = zeros(n,1);
for k = 1:n
    q_err = quaternion_ops.quat_error(q_target, q_hist(k,:)');
    if q_err(1) < 0, q_err = -q_err; end
    err_angle_deg(k) = 2 * rad2deg(acos(min(max(q_err(1),-1),1)));
end

h_wheel_mag = vecnorm(h_wheel_hist, 2, 2);

%% Report
fprintf('--- Disturbance rejection (pure PD, gravity-gradient active) ---\n');
fprintf('Initial attitude error: %.2f deg\n', err_angle_deg(1));
fprintf('Final attitude error:   %.4f deg  <-- expect NONZERO (this is the point)\n', err_angle_deg(end));
fprintf('Steady-state error (mean of last 20%% of run): %.4f deg\n', mean(err_angle_deg(round(0.8*n):end)));
fprintf('Max |h_wheel| reached:  %.5f N*m*s (%.1f%% of capacity)\n', ...
    max(h_wheel_mag), 100*max(h_wheel_mag)/h_max);

fprintf('\nCompare final error above to main_pointing.m''s 0.0000 deg (no disturbance).\n');
fprintf('A nonzero settled value here confirms P+D alone cannot null a persistent\n');
fprintf('disturbance torque -- next step adds an integral term to fix this.\n');

%% Plots
figure('Name', 'Disturbance Rejection (Pure PD)');

subplot(2,2,1);
plot(t, omega_hist, 'LineWidth', 1.3);
xlabel('Time (s)'); ylabel('\omega (rad/s)');
legend('\omega_x','\omega_y','\omega_z');
title('Angular velocity');
grid on;

subplot(2,2,2);
plot(t, err_angle_deg, 'LineWidth', 1.5, 'Color', [0.85 0.1 0.1]);
xlabel('Time (s)'); ylabel('Attitude error (deg)');
title('Pointing error -- watch for nonzero PLATEAU, not decay to 0');
grid on;

subplot(2,2,3);
plot(t, h_wheel_hist, 'LineWidth', 1.2);
hold on;
yline(h_max, '--k', 'h_{max}'); yline(-h_max, '--k');
xlabel('Time (s)'); ylabel('h_{wheel} (N m s)');
legend('h_x','h_y','h_z','Location','best');
title('Wheel momentum -- watch for steady DRIFT (integral of steady torque)');
grid on;

subplot(2,2,4);
plot(t, h_wheel_mag, 'LineWidth', 1.5);
hold on;
yline(h_max, '--k', 'h_{max}');
xlabel('Time (s)'); ylabel('|h_{wheel}| (N m s)');
title('Wheel momentum magnitude vs saturation limit');
grid on;

saveas(gcf, '../results/figures/disturbance_rejection_pd_only.png');
fprintf('\nFigure saved to results/figures/disturbance_rejection_pd_only.png\n');