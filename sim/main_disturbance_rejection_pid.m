%% MAIN_DISTURBANCE_REJECTION_PID.M
% Same scenario as main_disturbance_rejection.m (real gravity-gradient
% torque, same Kp/Kd, same target/initial conditions) with an integral
% term added. Purpose: show Ki drives the already-small PD-only residual
% (0.0009-0.0015 deg, see main_disturbance_rejection.m results) further
% toward true zero. This is a SMALL but HONEST claim -- gravity-gradient
% is genuinely weak for this spacecraft, so don't expect a dramatic
% before/after; expect the final error to drop roughly another order of
% magnitude or more, not a rescue from a large offset.

clear; clc; close all;
addpath('../src/dynamics');
addpath('../src/actuators');
addpath('../src/control');
addpath('../src/utils');
addpath('../src/environment');

%% Setup -- identical to main_disturbance_rejection.m except controller
I_body = inertia_properties();

omega0 = [0.02; -0.015; 0.025];
q0 = [1; 0; 0; 0];
h_wheel0 = [0; 0; 0];
e_int0 = [0; 0; 0];
state0 = [omega0; q0; h_wheel0; e_int0];

target_angle_deg = 45;
target_axis = [1;1;1] / norm([1;1;1]);
theta = deg2rad(target_angle_deg);
q_target = [cos(theta/2); target_axis * sin(theta/2)];

h_max = 0.01;
tau_max = 0.0006;

Kp = 3e-3;    % SAME as the PD-only run
Kd = 1.2e-2;  % SAME as the PD-only run
Ki = 2e-5;    % REVERTED from the 5e-6 test. Both runs showed a PERSISTENT,
              % non-shrinking oscillation -- initially read as a possible
              % Ki-induced limit cycle, but lowering Ki made the residual
              % WORSE (0.0007deg -> 0.0018deg), the opposite of what a
              % simple over-aggressive-Ki explanation predicts. Correct
              % explanation: q_target is fixed in the INERTIAL frame, so
              % gravity-gradient torque (which depends on the nadir
              % direction relative to the body) is periodic with orbital
              % motion, not constant/DC. A PID integral term nulls a DC
              % disturbance exactly but only ATTENUATES a periodic one --
              % higher Ki gives better attenuation (smaller residual
              % ripple), which is why this gain outperformed the lower one.
              % Exact rejection of the periodic component would require a
              % repetitive/resonant controller, out of scope here.
e_int_max = 5.0;   % anti-windup clamp (in units of accumulated qe_vec,
                   % dimensionless quaternion-vector-part-seconds)

control_fn = @(q, omega, e_int) pid_attitude_control_full( ...
    q, q_target, omega, e_int, Kp, Ki, Kd, e_int_max);

alt_km = 500;
inc_deg = 51.6;
tau_ext_fn = @(t, q) disturbance_torques(orbit_propagator(t, alt_km, inc_deg), q, I_body);

t_span = [0, 8000];   % extended from 2000s -- the last run showed the error
                       % dip to ~0.3-0.4deg then RISE back to ~0.7-0.8deg
                       % near the end of the window, not a clean settled
                       % plateau. Holding Ki fixed and only extending time
                       % isolates whether this is a slowly-damping
                       % oscillation (should keep shrinking) or a
                       % persistent limit cycle (would keep oscillating at
                       % roughly constant amplitude) -- can't tell which
                       % from a 2000s window alone

%% Integrate
opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);
[t, state] = ode45(@(t,s) attitude_state_derivative_pid( ...
    t, s, I_body, control_fn, h_max, tau_max, tau_ext_fn), t_span, state0, opts);

omega_hist   = state(:, 1:3);
q_hist       = state(:, 4:7);
h_wheel_hist = state(:, 8:10);
e_int_hist   = state(:, 11:13);

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
fprintf('--- Disturbance rejection: FULL PID (Ki=%.1e) vs PD-only baseline ---\n', Ki);
fprintf('Initial attitude error: %.2f deg\n', err_angle_deg(1));
fprintf('Final attitude error:   %.6f deg\n', err_angle_deg(end));
fprintf('Steady-state error (mean of last 20%% of run): %.6f deg\n', mean(err_angle_deg(round(0.8*n):end)));
tail_idx = round(0.5*n):n;   % second half, to look past the initial transient
fprintf('Peak-to-peak error amplitude, SECOND HALF of run: %.6f deg (min=%.6f, max=%.6f)\n', ...
    max(err_angle_deg(tail_idx)) - min(err_angle_deg(tail_idx)), ...
    min(err_angle_deg(tail_idx)), max(err_angle_deg(tail_idx)));
tail_idx2 = round(0.75*n):n;   % last quarter only
fprintf('Peak-to-peak error amplitude, LAST QUARTER of run:  %.6f deg (min=%.6f, max=%.6f)\n', ...
    max(err_angle_deg(tail_idx2)) - min(err_angle_deg(tail_idx2)), ...
    min(err_angle_deg(tail_idx2)), max(err_angle_deg(tail_idx2)));
fprintf('-- If the last-quarter amplitude is NOT clearly smaller than the second-half\n');
fprintf('   amplitude, this is a persistent oscillation, not a settling transient.\n');
fprintf('PD-only baseline (main_disturbance_rejection.m) was: 0.0009 deg steady-state\n');
fprintf('Max |h_wheel| reached: %.5f N*m*s (%.1f%% of capacity)\n', ...
    max(h_wheel_mag), 100*max(h_wheel_mag)/h_max);
fprintf('Max |e_int| reached:   %.4f (anti-windup limit = %.1f)\n', max(max(abs(e_int_hist))), e_int_max);

%% Plots
figure('Name', 'Disturbance Rejection: Full PID');

subplot(2,2,1);
plot(t, err_angle_deg, 'LineWidth', 1.5, 'Color', [0.85 0.1 0.1]);
xlabel('Time (s)'); ylabel('Attitude error (deg)');
title('Pointing error (full transient)');
grid on;

subplot(2,2,2);
plot(t(round(0.5*n):end), err_angle_deg(round(0.5*n):end), 'LineWidth', 1.5, 'Color', [0.1 0.6 0.85]);
xlabel('Time (s)'); ylabel('Attitude error (deg)');
title('Pointing error -- zoomed on settled region (second half of run)');
grid on;

subplot(2,2,3);
plot(t, e_int_hist, 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('e_{int} (accumulated error)');
legend('e_{int,x}','e_{int,y}','e_{int,z}');
title('Integral state (watch for windup vs settling)');
grid on;

subplot(2,2,4);
plot(t, h_wheel_hist, 'LineWidth', 1.2);
hold on;
yline(h_max, '--k', 'h_{max}'); yline(-h_max, '--k');
xlabel('Time (s)'); ylabel('h_{wheel} (N m s)');
legend('h_x','h_y','h_z','Location','best');
title('Wheel momentum');
grid on;

saveas(gcf, '../results/figures/disturbance_rejection_pid.png');
fprintf('\nFigure saved to results/figures/disturbance_rejection_pid.png\n');