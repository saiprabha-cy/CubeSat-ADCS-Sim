classdef quaternion_ops
    % QUATERNION_OPS  Core quaternion utility functions for the CubeSat ADCS simulator.
    %
    % Convention: q = [q0; q1; q2; q3], q0 = scalar part, [q1 q2 q3] = vector part.
    % q represents a rotation from the INERTIAL frame to the BODY frame
    % (i.e. v_body = quaternion_ops.quat_rotate(q, v_inertial)) unless stated otherwise.
    %
    % Usage from any other file in this project:
    %   q3 = quaternion_ops.quat_multiply(q1, q2);
    %   q  = quaternion_ops.quat_normalize(q);
    %
    % Implemented as a class with Static methods (not separate function files)
    % so the whole quaternion toolkit stays in one place, matching the project's
    % module-per-concern folder structure while remaining fully callable.

    methods (Static)

        function q_out = quat_multiply(q, r)
            % Hamilton product: q_out = q ⊗ r
            q0 = q(1); qv = q(2:4);
            r0 = r(1); rv = r(2:4);

            q_out = zeros(4,1);
            q_out(1)   = q0*r0 - dot(qv, rv);
            q_out(2:4) = q0*rv + r0*qv + cross(qv, rv);
        end

        function q_conj = quat_conjugate(q)
            % Valid as the inverse ONLY if q is unit norm (true for attitude
            % quaternions as long as you normalize every step -- see quat_normalize).
            q_conj = [q(1); -q(2); -q(3); -q(4)];
        end

        function q_n = quat_normalize(q)
            n = norm(q);
            if n < 1e-12
                error('quaternion_ops:badNorm', ...
                    'quat_normalize: quaternion norm ~0, cannot normalize (check upstream integration).');
            end
            q_n = q(:) / n;
        end

        function v_out = quat_rotate(q, v)
            % Rotate 3-vector v from inertial frame into body frame using q.
            % Implemented via the double quaternion product: v' = q ⊗ [0; v] ⊗ q_conj
            q = quaternion_ops.quat_normalize(q);
            v_quat = [0; v(:)];
            v_rot_quat = quaternion_ops.quat_multiply( ...
                quaternion_ops.quat_multiply(q, v_quat), ...
                quaternion_ops.quat_conjugate(q));
            v_out = v_rot_quat(2:4);
        end

        function euler = quat_to_euler(q)
            % Returns [roll; pitch; yaw] in radians. 3-2-1 (yaw-pitch-roll) convention.
            % DEBUG/PLOTTING ONLY -- do not feed this back into the control law;
            % it exists purely so you have a human-readable attitude to eyeball on a plot.
            q = quaternion_ops.quat_normalize(q);
            q0=q(1); q1=q(2); q2=q(3); q3=q(4);

            roll  = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2));
            sinp  = 2*(q0*q2 - q3*q1);
            sinp  = max(min(sinp, 1), -1); % clamp for numerical safety near +/-90 deg
            pitch = asin(sinp);
            yaw   = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2));

            euler = [roll; pitch; yaw];
        end

        function q_dot = quat_derivative(q, w)
            % Propagation equation: q_dot = 1/2 * Omega(w) * q
            % w = [wx; wy; wz] angular velocity in body frame (rad/s)
            wx = w(1); wy = w(2); wz = w(3);

            Omega = [ 0   -wx  -wy  -wz;
                      wx   0    wz  -wy;
                      wy  -wz   0    wx;
                      wz   wy  -wx   0 ];

            q_dot = 0.5 * Omega * q(:);
        end

        function q_err = quat_error(q_target, q)
            % Attitude error quaternion: how far 'q' is from 'q_target'.
            % Vector part of q_err is ~proportional to small-angle attitude error --
            % this is what the PD controller in pid_attitude_control.m consumes directly.
            q_err = quaternion_ops.quat_multiply(quaternion_ops.quat_conjugate(q_target), q);
            q_err = quaternion_ops.quat_normalize(q_err);
        end

    end
end