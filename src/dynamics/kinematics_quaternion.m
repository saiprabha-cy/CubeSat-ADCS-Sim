function q_dot = kinematics_quaternion(q, omega)
% KINEMATICS_QUATERNION  Attitude kinematics: how the quaternion evolves
% given the current angular velocity.
%
%   q_dot = kinematics_quaternion(q, omega)
%
%   q     : [q0; q1; q2; q3] current attitude quaternion (inertial->body)
%   omega : [wx; wy; wz] angular velocity in body frame (rad/s)
%
%   This is a thin wrapper around quaternion_ops.quat_derivative -- kept as
%   its own file so dynamics/ (rotational EOM) and kinematics (attitude
%   representation) stay conceptually and physically separate, even though
%   they're coupled through the same state vector during integration.
%
%   IMPORTANT: this returns the raw derivative only. The unit-norm
%   constraint on q is NOT enforced here -- normalize q with
%   quaternion_ops.quat_normalize AFTER each integration step (see
%   attitude_state_derivative.m / the main_*.m run scripts for where
%   that renormalization happens).

    q_dot = quaternion_ops.quat_derivative(q, omega);
end