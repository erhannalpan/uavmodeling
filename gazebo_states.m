% Readable copy extracted from the uploaded SLX MATLAB Function block.
% The SLX executes its embedded copy; editing this file does not edit that block.
% See README.md.
function [x_gz, t_gz] = gazebo_states(position, orientation, twist, stamp)

t_gz = double(stamp.sec) + double(stamp.nanosec)*1e-9;
% Gazebo odometry -> quadrotor model states
%
% Output order:
% [pn pe h u v w phi theta psi p q r]'
%
% Coordinate assumptions:
% Gazebo body: X forward, Y left, Z up.
% Model body:  X forward, Y right, Z down.
% Define reference north = Gazebo world +X,
% east = Gazebo world -Y, and altitude = world +Z.
% Odometry twist is expressed in the Gazebo body frame.

x_gz = zeros(12,1);

%% Read quaternion describing Gazebo body orientation in world
qw = double(orientation.w);
qx = double(orientation.x);
qy = double(orientation.y);
qz = double(orientation.z);

% Return zeros for blank messages before odometry arrives.
% These zeros must not be treated as valid measured states.
qn = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);

if qn < 1e-12
    return;
end

% Normalize quaternion
qw = qw/qn;
qx = qx/qn;
qy = qy/qn;
qz = qz/qn;

%% Rotation matrix: Gazebo body coordinates -> Gazebo world coordinates
R = [1-2*(qy*qy+qz*qz), 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy);
    2*(qx*qy+qw*qz), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qw*qx);
    2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), 1-2*(qx*qx+qy*qy)];

%% Convert body and reference coordinate conventions
D = diag([1 -1 -1]);

% Rotation matrix: model body -> north-east-down reference
Rm = D*R*D;

%% Reference-frame position, in metres
x_gz(1) =  double(position.x);  % pn: north
x_gz(2) = -double(position.y);  % pe: east
x_gz(3) =  double(position.z);  % h: altitude, positive upward

%% Linear velocity expressed in model body axes, in m/s
x_gz(4) =  double(twist.linear.x);  % u: forward
x_gz(5) = -double(twist.linear.y);  % v: right
x_gz(6) = -double(twist.linear.z);  % w: down

%% Euler attitude angles relative to the reference frame, in radians
% ZYX convention: yaw, then pitch, then roll.
% These are NOT the integrals of body angular rates p, q, r.
x_gz(7) = atan2(Rm(3,2), Rm(3,3));          % phi: roll angle
x_gz(8) = asin(max(-1, min(1, -Rm(3,1)))); % theta: pitch angle
x_gz(9) = atan2(Rm(2,1), Rm(1,1));          % psi: yaw angle

%% Angular velocity expressed in model body axes, in rad/s
% Components about the current body axes, not Euler-angle derivatives.
x_gz(10) =  double(twist.angular.x);  % p: about body X
x_gz(11) = -double(twist.angular.y);  % q: about body Y
x_gz(12) = -double(twist.angular.z);  % r: about body Z

end

