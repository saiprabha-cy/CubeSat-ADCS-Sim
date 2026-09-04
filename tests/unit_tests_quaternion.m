function unit_tests_quaternion()
% UNIT_TESTS_QUATERNION
% Sanity checks for quaternion_ops.m using cases with known analytical answers.
% Run this BEFORE writing rigid_body_dynamics.m -- if quaternion math is wrong,
% every downstream result (detumble, pointing, everything) will silently be wrong
% in a way that's much harder to diagnose once physics and control are layered on top.
%
% Run with: unit_tests_quaternion
% Expect: "ALL TESTS PASSED" at the end. Any FAIL prints details and stops.

    tol = 1e-9;
    n_pass = 0;
    n_total = 0;

    % --- Test 1: Identity quaternion does nothing ---------------------------
    n_total = n_total + 1;
    q_identity = [1; 0; 0; 0];
    v = [1; 2; 3];
    v_rot = quaternion_ops.quat_rotate(q_identity, v);
    if check(v_rot, v, tol, 'Test 1: identity rotation')
        n_pass = n_pass + 1;
    end

    % --- Test 2: 90 deg rotation about Z maps X-axis to Y-axis --------------
    n_total = n_total + 1;
    theta = pi/2;
    q_z90 = [cos(theta/2); 0; 0; sin(theta/2)];
    v_x = [1; 0; 0];
    v_rot = quaternion_ops.quat_rotate(q_z90, v_x);
    expected = [0; 1; 0];
    if check(v_rot, expected, 1e-6, 'Test 2: 90 deg Z-rotation of X-axis -> Y-axis')
        n_pass = n_pass + 1;
    end

    % --- Test 3: quaternion multiplied by its conjugate = identity ----------
    n_total = n_total + 1;
    q = quaternion_ops.quat_normalize([0.5; 0.5; 0.5; 0.5]);
    q_conj = quaternion_ops.quat_conjugate(q);
    q_result = quaternion_ops.quat_multiply(q, q_conj);
    expected = [1; 0; 0; 0];
    if check(q_result, expected, tol, 'Test 3: q ⊗ q_conjugate = identity')
        n_pass = n_pass + 1;
    end

    % --- Test 4: normalize returns unit norm ---------------------------------
    n_total = n_total + 1;
    q_unnorm = [2; 0; 0; 0];
    q_n = quaternion_ops.quat_normalize(q_unnorm);
    if check(norm(q_n), 1, tol, 'Test 4: normalize -> unit norm')
        n_pass = n_pass + 1;
    end

    % --- Test 5: quat_error(q, q) = identity (zero attitude error) ----------
    n_total = n_total + 1;
    q_rand = quaternion_ops.quat_normalize([0.7; 0.1; 0.2; 0.3]);
    q_err = quaternion_ops.quat_error(q_rand, q_rand);
    expected = [1; 0; 0; 0];
    if check(q_err, expected, tol, 'Test 5: quat_error(q,q) = identity')
        n_pass = n_pass + 1;
    end

    % --- Test 6: quat_to_euler(identity) = [0;0;0] ---------------------------
    n_total = n_total + 1;
    euler = quaternion_ops.quat_to_euler(q_identity);
    if check(euler, [0;0;0], tol, 'Test 6: identity quaternion -> zero Euler angles')
        n_pass = n_pass + 1;
    end

    % --- Test 7: quat_derivative of identity with zero omega = zero ---------
    n_total = n_total + 1;
    q_dot = quaternion_ops.quat_derivative(q_identity, [0;0;0]);
    if check(q_dot, [0;0;0;0], tol, 'Test 7: zero angular velocity -> zero q_dot')
        n_pass = n_pass + 1;
    end

    % --- Summary --------------------------------------------------------------
    fprintf('\n%d / %d tests passed.\n', n_pass, n_total);
    if n_pass == n_total
        fprintf('ALL TESTS PASSED.\n');
    else
        error('unit_tests_quaternion:failures', '%d test(s) failed -- fix before proceeding.', n_total - n_pass);
    end
end

function result = check(actual, expected, tol, label)
    result = all(abs(actual(:) - expected(:)) < tol);
    if result
        fprintf('[PASS] %s\n', label);
    else
        fprintf('[FAIL] %s\n', label);
        fprintf('       expected: %s\n', mat2str(expected(:)', 6));
        fprintf('       actual:   %s\n', mat2str(actual(:)', 6));
    end
end