%% MAIN_DETUMBLE.M
% Build-order step 3: run the B-dot detumbling controller against the
% verified torque-free dynamics, confirm |omega| decays from an initial
% tumble toward zero.

clear; clc; close all;
addpath('../src/dynamics');
addpath('../src/environment');
addpath('../src/control');
addpath('../src/actuators');
addpath('../src/utils');

%% Setup
I = inertia_properties();                 % default 3U CubeSat
omega0 = [0.8; -0.6; 0.4];                 % rad/s -- a genuinely tumbling initial state
                                            % (post-deployment tip-off rates are commonly
                                            % on the order of a few deg/s to ~tens of deg/s;
                                            % this is an intentionally aggressive test case)
q0 = [1; 0; 0; 0];
state0 = [omega0; q0];

alt_km  = 500;
inc_deg = 51.6;
tilt_deg = 11.7;

k = 8e3;        % B-dot gain -- reduced from 5e4. At |omega|~1 rad/s, B~3e-5 T,
                % Bdot ~ 3e-5 T/s, so m_cmd ~ k*3e-5 ~ 0.24 A*m^2 -- near but
                % not permanently pinned at m_max=0.2, restoring proportional
                % (not pure bang-bang) behavior for most of the trajectory
m_max = 0.2;    % A*m^2 per axis

t_span = [0, 21000];   % small extension from the 17000s run (which reached
                        % 0.082 rad/s, close to the 0.05 threshold) -- B-dot
                        % authority shrinks as omega shrinks (torque ~ omega),
                        % so convergence is asymptotic; this tests whether a
                        % modest extension is enough to cross 0.05 rad/s

%% Torque function: full B-dot control chain, called at every ode45 evaluation
% (local function 'detumble_tau_fn' is defined at the END of this file --
% MATLAB requires local functions in a script to appear after all
% script-level statements, not interspersed among them)
tau_fn = @(t, w, q) detumble_tau_fn(t, w, q, alt_km, inc_deg, tilt_deg, k, m_max);

%% Integrate
opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);
[t, state] = ode45(@(t,s) attitude_state_derivative(t, s, I, tau_fn), t_span, state0, opts);

omega_hist = state(:, 1:3);
q_hist     = state(:, 4:7);

for kk = 1:length(t)
    q_hist(kk,:) = quaternion_ops.quat_normalize(q_hist(kk,:)');
end

omega_mag = vecnorm(omega_hist, 2, 2);

%% Reconstruct commanded moment / torque history for plotting (post-hoc, for inspection)
n = length(t);
m_actual_hist = zeros(n,3);
tau_hist = zeros(n,3);
for kk = 1:n
    r_eci  = orbit_propagator(t(kk), alt_km, inc_deg);
    B_eci  = magnetic_field_model(r_eci, tilt_deg);
    B_body = quaternion_ops.quat_rotate(q_hist(kk,:)', B_eci);
    m_cmd  = detumble_bdot(omega_hist(kk,:)', B_body, k);
    [tau_b, m_a] = magnetorquer_model(m_cmd, B_body, m_max);
    m_actual_hist(kk,:) = m_a';
    tau_hist(kk,:) = tau_b';
end

%% Report
omega_start = omega_mag(1);
omega_end   = omega_mag(end);
fprintf('--- Detumble results ---\n');
fprintf('Initial |omega|: %.4f rad/s (%.2f deg/s)\n', omega_start, rad2deg(omega_start));
fprintf('Final   |omega|: %.6f rad/s (%.4f deg/s)\n', omega_end, rad2deg(omega_end));
fprintf('Reduction: %.2f%%\n', 100*(omega_start-omega_end)/omega_start);

% time to reach some threshold, e.g. 0.05 rad/s (~2.9 deg/s) -- a common
% "detumbled enough to hand off to fine pointing" threshold
threshold = 0.05;
idx = find(omega_mag < threshold, 1, 'first');
if isempty(idx)
    fprintf('Did NOT reach %.3f rad/s threshold within %.0f s -- increase k or t_span.\n', threshold, t_span(2));
else
    fprintf('Reached |omega| < %.3f rad/s at t = %.1f s\n', threshold, t(idx));
end

%% Plots
figure('Name', 'B-dot Detumble Simulation');

subplot(2,2,1);
plot(t, omega_hist, 'LineWidth', 1.3);
xlabel('Time (s)'); ylabel('\omega (rad/s)');
legend('\omega_x','\omega_y','\omega_z');
title('Angular velocity components');
grid on;

subplot(2,2,2);
plot(t, omega_mag, 'LineWidth', 1.5, 'Color', [0.85 0.1 0.1]);
xlabel('Time (s)'); ylabel('|\omega| (rad/s)');
title('Angular velocity magnitude (should decay toward 0)');
grid on;

subplot(2,2,3);
plot(t, m_actual_hist, 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('m (A m^2)');
legend('m_x','m_y','m_z');
title(sprintf('Commanded dipole moment (saturation limit = %.2f A m^2)', m_max));
ylim([-m_max*1.2, m_max*1.2]);
grid on;

subplot(2,2,4);
plot(t, tau_hist, 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('\tau (N m)');
legend('\tau_x','\tau_y','\tau_z');
title('Control torque applied');
grid on;

saveas(gcf, '../results/figures/detumble_result.png');
fprintf('\nFigure saved to results/figures/detumble_result.png\n');

%% --- Local functions (must be at end of script file per MATLAB rules) ---
function tau = detumble_tau_fn(t, omega, q, alt_km, inc_deg, tilt_deg, k, m_max)
    r_eci  = orbit_propagator(t, alt_km, inc_deg);
    B_eci  = magnetic_field_model(r_eci, tilt_deg);
    B_body = quaternion_ops.quat_rotate(q, B_eci);

    m_cmd = detumble_bdot(omega, B_body, k);
    [tau, ~] = magnetorquer_model(m_cmd, B_body, m_max);
end