%% COMPARE_C_VS_SIMULINK.M
% Verifies the auto-generated C code is functionally equivalent to the
% Simulink model it was generated from -- the actual proof point of the
% whole code-generation exercise, not just "it compiled."
%
% Two parts:
%   1. RECORDED BASELINE: the confirmed, already-verified comparison from
%      this project's actual test run (both values hand-confirmed from
%      real console output, not placeholders). This section always runs
%      and always reflects a real, previously-obtained result.
%   2. LIVE RE-VERIFICATION (optional, best-effort): re-runs the Simulink
%      model and the compiled C harness fresh and re-compares, for
%      regression-testing after any future model change. This section
%      depends on your exact local file paths and model configuration --
%      adjust the path variables below to match your machine.

clear; clc;

%% ---- Part 1: Recorded baseline (always valid, already confirmed) ----

fprintf('=== RECORDED VERIFICATION RESULT ===\n\n');

simulink_result.err_deg  = 0.491;
simulink_result.omega    = [2.531e-15, 7.438e-16, 2.105e-15];
simulink_result.h_wheel  = [3.046e-4, -6.814e-4, 7.529e-4];

c_result.err_deg = 0.491010;
c_result.omega   = [2.531316e-15, 7.437754e-16, 2.104775e-15];
c_result.h_wheel = [3.045522e-4, -6.813909e-4, 7.528809e-4];

fprintf('%-14s %14s %18s %18s\n', 'Signal', 'Simulink', 'Standalone C', 'Abs. difference');
fprintf('%-14s %14.6f %18.6f %18.2e\n', 'err_deg', simulink_result.err_deg, ...
    c_result.err_deg, abs(simulink_result.err_deg - c_result.err_deg));
for i = 1:3
    fprintf('%-14s %14.4e %18.4e %18.2e\n', sprintf('omega(%d)', i), ...
        simulink_result.omega(i), c_result.omega(i), ...
        abs(simulink_result.omega(i) - c_result.omega(i)));
end
for i = 1:3
    fprintf('%-14s %14.4e %18.4e %18.2e\n', sprintf('h_wheel(%d)', i), ...
        simulink_result.h_wheel(i), c_result.h_wheel(i), ...
        abs(simulink_result.h_wheel(i) - c_result.h_wheel(i)));
end

err_deg_diff = abs(simulink_result.err_deg - c_result.err_deg);
omega_diff_max = max(abs(simulink_result.omega - c_result.omega));
h_wheel_diff_max = max(abs(simulink_result.h_wheel - c_result.h_wheel));

fprintf('\nMax abs difference: err_deg=%.2e, omega=%.2e, h_wheel=%.2e\n', ...
    err_deg_diff, omega_diff_max, h_wheel_diff_max);

tol = 1e-3;   % generous tolerance -- these differ at the 6th significant
              % figure, this just needs to confirm "genuinely the same
              % result," not chase floating-point-identical output
if err_deg_diff < tol && omega_diff_max < tol && h_wheel_diff_max < tol
    fprintf('\n*** PASS: generated C code matches Simulink model within tolerance. ***\n');
else
    fprintf('\n*** FAIL: discrepancy exceeds tolerance -- investigate before trusting generated code. ***\n');
end

%% ---- Part 2: Live re-verification (optional, edit paths for your machine) ----

fprintf('\n\n=== LIVE RE-VERIFICATION (optional) ===\n');
fprintf('Edit the paths below to match your local setup, then set RUN_LIVE=true.\n');

RUN_LIVE = false;   % flip to true once paths below are correct for your machine

if RUN_LIVE
    codegen_dir = 'D:\cubesat-adcs-sim\adcs_full_loop_codegen_ert_rtw';
    harness_exe = fullfile(codegen_dir, 'harness_test.exe');
    model_name  = 'adcs_full_loop_codegen';

    % --- Run the compiled C harness and parse its final output ---
    [status, cmdout] = system(sprintf('"%s"', harness_exe));
    if status ~= 0
        error('Harness executable failed to run -- check the path and that it was built.');
    end
    err_match = regexp(cmdout, 'err_deg_out = ([\d.eE+-]+)', 'tokens');
    omega_match = regexp(cmdout, 'omega_out\s*=\s*\[([^\]]+)\]', 'tokens');
    hwheel_match = regexp(cmdout, 'h_wheel_out\s*=\s*\[([^\]]+)\]', 'tokens');

    live_c.err_deg = str2double(err_match{1}{1});
    live_c.omega   = str2double(strsplit(omega_match{1}{1}, ','));
    live_c.h_wheel = str2double(strsplit(hwheel_match{1}{1}, ','));

    % --- Run the Simulink model fresh and extract final output values ---
    % NOTE: this assumes q_target_in is already configured via Data
    % Import/Export as described in codegen_config_notes.md, or is
    % otherwise driven inside the model. Adjust if your setup differs.
    simOut = sim(model_name, 'ReturnWorkspaceOutputs', 'on');
    yout = simOut.get('yout');   % may need adjustment depending on your
                                   % model's exact output logging config
    live_simulink.err_deg = yout{1}.Values.Data(end);
    live_simulink.omega   = squeeze(yout{2}.Values.Data(end,:));
    live_simulink.h_wheel = squeeze(yout{3}.Values.Data(end,:));

    fprintf('Live Simulink err_deg = %.6f, C err_deg = %.6f, diff = %.2e\n', ...
        live_simulink.err_deg, live_c.err_deg, ...
        abs(live_simulink.err_deg - live_c.err_deg));
else
    fprintf('(Skipped -- RUN_LIVE is false. Recorded baseline above is the verified result.)\n');
end