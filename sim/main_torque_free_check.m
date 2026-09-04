%% MAIN_TORQUE_FREE_CHECK.M
% Verification checkpoint (build-order step 2): integrate the dynamics with
% ZERO applied torque and confirm two physical invariants hold:
%   1. Rotational kinetic energy: T = 0.5 * omega' * I * omega  -- CONSTANT
%   2. Angular momentum magnitude: |L| = |I*omega|              -- CONSTANT
%      (the VECTOR L is constant only in the inertial frame, but its
%       MAGNITUDE is frame-invariant, so checking |L| in body frame is a
%       valid and simple test)
% If either drifts significantly, the dynamics implementation has a bug --
% fix it here before writing any controller, since a wrong physics model
% is far harder to debug once a control law is layered on top of it.

clear; clc; close all;
addpath('../src/dynamics');
addpath('../src/utils');

%% Setup
I = inertia_properties();                  % default 3U CubeSat inertia
omega0 = [0.5; -0.3; 0.2];                  % rad/s, asymmetric tumble -- deliberately
                                             % NOT aligned with a principal axis, so the
                                             % gyroscopic coupling term is actually exercised
q0 = [1; 0; 0; 0];                          % start aligned with inertial frame
state0 = [omega0; q0];

tau_free = @(t, w, q) [0; 0; 0];            % torque-free: this IS the test condition

t_span = [0, 60];                           % seconds

%% Integrate
opts = odeset('RelTol', 1e-10, 'AbsTol', 1e-12);  % tight tolerances -- this is a
                                                    % verification run, not a
                                                    % production sim; we want to
                                                    % isolate MODEL error from
                                                    % INTEGRATOR error
[t, state] = ode45(@(t,s) attitude_state_derivative(t, s, I, tau_free), t_span, state0, opts);

omega_hist = state(:, 1:3);
q_hist     = state(:, 4:7);

%% Renormalize quaternions post-integration (see attitude_state_derivative.m note)
q_norm_drift = zeros(size(t));
for k = 1:length(t)
    q_norm_drift(k) = norm(q_hist(k,:));
    q_hist(k,:) = quaternion_ops.quat_normalize(q_hist(k,:)');
end

%% Compute invariants over time
n = length(t);
KE = zeros(n,1);
L_mag = zeros(n,1);
for k = 1:n
    w = omega_hist(k,:)';
    L = I * w;
    KE(k) = 0.5 * w' * L;
    L_mag(k) = norm(L);
end

%% Report drift
KE_drift_pct    = 100 * (max(KE) - min(KE)) / mean(KE);
Lmag_drift_pct  = 100 * (max(L_mag) - min(L_mag)) / mean(L_mag);
qnorm_max_drift = max(abs(q_norm_drift - 1));

fprintf('--- Torque-free invariant check ---\n');
fprintf('Kinetic energy:      mean = %.6e J, drift = %.6f %%\n', mean(KE), KE_drift_pct);
fprintf('Angular momentum |L|: mean = %.6e, drift = %.6f %%\n', mean(L_mag), Lmag_drift_pct);
fprintf('Quaternion norm max drift (pre-renormalize): %.3e\n', qnorm_max_drift);

if KE_drift_pct < 0.01 && Lmag_drift_pct < 0.01
    fprintf('\nPASS: energy and angular momentum magnitude are conserved within tolerance.\n');
    fprintf('Dynamics implementation verified -- safe to proceed to actuators/control.\n');
else
    fprintf('\nFAIL: drift exceeds 0.01%% -- do not proceed until this is resolved.\n');
    fprintf('Check: sign errors in rigid_body_dynamics.m, incorrect I, or tau_free not actually zero.\n');
end

%% Plots
figure('Name', 'Torque-Free Verification');

subplot(2,2,1);
plot(t, omega_hist, 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\omega (rad/s)');
legend('\omega_x','\omega_y','\omega_z');
title('Angular velocity (should oscillate but never grow/decay)');
grid on;

subplot(2,2,2);
plot(t, KE, 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Kinetic energy (J)');
title(sprintf('Rotational KE (drift = %.4f%%)', KE_drift_pct));
grid on;

subplot(2,2,3);
plot(t, L_mag, 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('|L| (kg m^2/s)');
title(sprintf('Angular momentum magnitude (drift = %.4f%%)', Lmag_drift_pct));
grid on;

subplot(2,2,4);
plot(t, q_norm_drift, 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('|q| pre-renormalize');
title('Quaternion norm drift from integration');
grid on;

saveas(gcf, '../results/figures/torque_free_check.png');
fprintf('\nFigure saved to results/figures/torque_free_check.png\n');