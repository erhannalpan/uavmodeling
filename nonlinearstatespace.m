% Readable copy extracted from the uploaded SLX MATLAB Function block.
% The SLX executes its embedded copy; editing this file does not edit that block.
% See README.md.
function xdot = nonlinearstatespace(x,uc,P)
% Nonlinear quadrotor state-space model
%
% x  = [pn pe h u v w phi theta psi p q r]'
% uc = [F tau_phi tau_theta tau_psi]'
% P  = [m g Jx Jy Jz]'

%% Parameters
m  = P(1);
g  = P(2);
Jx = P(3);
Jy = P(4);
Jz = P(5);

%% Generalized control inputs
F         = uc(1,1);
tau_phi   = uc(2,1);
tau_theta = uc(3,1);
tau_psi   = uc(4,1);

%% State variables
u     = x(4,1);
v     = x(5,1);
w     = x(6,1);
phi   = x(7,1);
theta = x(8,1);
psi   = x(9,1);
p     = x(10,1);
q     = x(11,1);
r     = x(12,1);

%% Useful trigonometric quantities
cphi   = cos(phi);
sphi   = sin(phi);
ctheta = cos(theta);
stheta = sin(theta);
cpsi   = cos(psi);
spsi   = sin(psi);

%% Position kinematics: inertial position rates
pn_dot = ctheta*cpsi*u ...
       + (sphi*stheta*cpsi-cphi*spsi)*v ...
       + (cphi*stheta*cpsi+sphi*spsi)*w;

pe_dot = ctheta*spsi*u ...
       + (sphi*stheta*spsi+cphi*cpsi)*v ...
       + (cphi*stheta*spsi-sphi*cpsi)*w;

h_dot = stheta*u ...
      - sphi*ctheta*v ...
      - cphi*ctheta*w;

%% Translational dynamics in body coordinates
u_dot = r*v-q*w-g*stheta;

v_dot = p*w-r*u+g*ctheta*sphi;

w_dot = q*u-p*v+g*ctheta*cphi-F/m;

%% Euler-angle kinematics
phi_dot = p ...
        + q*sphi*tan(theta) ...
        + r*cphi*tan(theta);

theta_dot = q*cphi-r*sphi;

psi_dot = (q*sphi+r*cphi)/ctheta;

%% Rotational rigid-body dynamics
p_dot = ((Jy-Jz)/Jx)*q*r+tau_phi/Jx;

q_dot = ((Jz-Jx)/Jy)*p*r+tau_theta/Jy;

r_dot = ((Jx-Jy)/Jz)*p*q+tau_psi/Jz;

%% Nonlinear state equation
xdot = [pn_dot;
        pe_dot;
        h_dot;
        u_dot;
        v_dot;
        w_dot;
        phi_dot;
        theta_dot;
        psi_dot;
        p_dot;
        q_dot;
        r_dot];
end