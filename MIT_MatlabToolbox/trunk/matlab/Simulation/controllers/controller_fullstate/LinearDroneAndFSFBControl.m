%% Linearization of drone dynamics about hover & full-state feedback design
% ===============================
% AUTHOR Fabian Riether
% CREATE DATE 2015/08/25
% PURPOSE This code assists in linearizing drone dynamics and designing
% full-state feedback controls
% SPECIAL NOTES
% ===============================
% Change History
%  2015/08/25 created
% ==================================

%Load drone parameters from RoboticsToolbox
mdl_quadrotor


%% 1.1) Simplified Dynamics

%symbolic variables
syms Pxw Pyw Pzw yaw pitch roll dpx dpy dpz p q r T tauy taup taur;
symsvector  = [Pxw; Pyw; Pzw ;yaw ;pitch ;roll ;dpx ;dpy ;dpz ;p ;q ;r ;T ;tauy ;taup ;taur];

%Transform inertia from RTB frame to RS frame
J           = ([cos(pi/4) -sin(pi/4) 0; sin(pi/4) cos(pi/4) 0; 0 0 1]'*quad.J*[cos(pi/4) -sin(pi/4) 0; sin(pi/4) cos(pi/4) 0; 0 0 1]);

%Define Rotation matrices
Ryaw = [
	[ cos(yaw), -sin(yaw), 0],
	[ sin(yaw),  cos(yaw), 0],
	[        0,         0, 1]
];

Rpitch = [
	[  cos(pitch), 0, sin(pitch)],
	[           0, 1,          0],
	[ -sin(pitch), 0, cos(pitch)]
];

Rroll = [
	[ 1,         0,          0],
	[ 0, cos(roll), -sin(roll)],
	[ 0, sin(roll),  cos(roll)]
];

Body2Global = Ryaw*Rpitch*Rroll;
Global2Body = simplify(Body2Global^-1);

%inverted Wronskian to transform body rates p-q-r to euler rates yaw pitch roll
iW = ...
    [0        sin(roll)          cos(roll);             
     0        cos(roll)*cos(pitch) -sin(roll)*cos(pitch);
     cos(pitch) sin(roll)*sin(pitch) cos(roll)*sin(pitch)] / cos(pitch);

%%Linearization Point = Hover
%-----------
state_equil = [0; 0; -1.5; 0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ]; %x_eq
input_equil = [-quad.g*quad.M ;0 ;0 ;0];		%u_eq
equil       = [state_equil; input_equil];

%%Dynamics
%----------      
%P dot      
P_dot           = simplify(Body2Global*[dpx;dpy;dpz]);
P_dot_jacobian  = jacobian(P_dot,symsvector);
P_dot_jacobian_eql = subs(P_dot_jacobian,symsvector,equil);

%O dot      
O_dot           = iW*[p;q;r];
O_dot_jacobian  = jacobian(O_dot,symsvector);
O_dot_jacobian_eql = subs(O_dot_jacobian,symsvector,equil);

%p ddot      
p_ddot          = Global2Body*[0;0;quad.g] + T/quad.M*[0;0;1] -cross(transpose([p,q,r]),transpose([dpx,dpy,dpz]));
p_ddot_jacobian = jacobian(p_ddot,symsvector);
p_ddot_jacobian_eql = subs(p_ddot_jacobian,symsvector,equil);
%o ddot      
o_ddot          = inv(J)*([taur; taup; tauy] - cross([p;q;r],J*[p;q;r]));
o_ddot_jacobian = jacobian(o_ddot,symsvector);
o_ddot_jacobian_eql = subs(o_ddot_jacobian,symsvector,equil);

%Dynamics matrix
%---------- 

matrixAB = [P_dot_jacobian_eql;O_dot_jacobian_eql;p_ddot_jacobian_eql;o_ddot_jacobian_eql];
A = double(matrixAB(1:12,1:12))
B = double(matrixAB(1:12,13:16))
%Note that with linearization, the resulting LTI system with states
%x_linSys and u_linSys runs around equilibrium conditions and NOT in the orginal state x_nonlinearSys and input space.
%It is x_nonlinearSys = x_eq + x_linearizedSys. Thus, x0_linearizedSys = x0_nonlinear - x_eq; 
%For the input, it is u_nonlinearSys = u_eq + x_linearizedSys.

%Eigenvalues are all zeros as it is a multiple double integrator (mechanical system with no constaints (drone flies freely in all 6 DOFs)

%% 1.2) Linearizing Full Nonlinear Simulink Model (the model from Robotics Toolbox)
%use Simulation/controllers/controller_fullstate/linearizeDrone.slx and Simulink's ControlDesign/Linear Analysis
load('LinearizedComplexDrone.mat');
A_fullnlDrone = linsys1.c*linsys1.a*inv(linsys1.c);
B_fullnlDrone = linsys1.c*linsys1.b;

%Eigenvalues are: 5 zeros (integrators), 3 stable modes, 2 unstable oscillatory modes
%Note that if we run the drone without motion in x,y,yaw-direction, the
%simple and the complex model behave the same way (differences in matrices
%relate to x-y-yaw-motion only)

%% 2.0) Load Full-state Feedback Controller derived from the PIDtoW-controller
%(see linearizePID2W.m)
K_pid =  [0 0 0.425862895347363 0 0 0 0 0 0.248420022285962 0 0 0;0 0 0 0.0102792311510984 0 0 0 0 0 0 0 0.00308376934532953;-0.00425177438846976 0 0 0 0.013286794963968 0 -0.00106294359711744 0 0 0 0.0017715726618624 0;0 0.00834853616902657 0 0 0 0.028788055755264 0 0.00259092501797376 0 0.00664339748198401 0 0];

% Generate c-code ready format for copy-paste straight into src-files rsedu_control.c
K_pid_ccode_string = sprintf('%E,' , K_pid(:));
K_pid_ccode_string = ['{ ' K_pid_ccode_string(1:end-1) ' }']

%% 2.1) Designing Full-state Feedback Controllers with Simplified Dynamics Model (1.1) via Pole Placement

% Find states to decouple
[V,J]   = jordan(A);
Veig_nrm = diag(1./sum(V,1))*V; % decoupled system will have a new state-vector x_dec = inv(Veig_nrm)*x

% System matrices of decoupled system
A_dec   = inv(Veig_nrm)*A*Veig_nrm;
B_dec   = inv(Veig_nrm)*B;

% Define decoupled subsystems
A_dec_x   = A_dec(1:4,1:4);
B_dec_x   = B_dec(1:4,1:4);

A_dec_y   = A_dec(7:10,7:10);
B_dec_y   = B_dec(7:10,1:4);

A_dec_z = A_dec(5:6,5:6);
B_dec_z = B_dec(5:6,1:4);

A_dec_yaw = A_dec(11:12,11:12);
B_dec_yaw = B_dec(11:12,1:4);

% Now place your own poles for the decoupled subsystems separately

xpoles      = [-9+6i;-9-6i;-0.18+1.8i;-0.18-1.8i];
ypoles      = [-60;-4;-0.1+2i;-0.1-2i];     
yawpoles    = [-3;-3.1];
%zpoles    = [-2;-2.1];               % Play around with poles here: Slow poles [-2;-2.1], Fast poles [-5;-5.1];
zpoles    = [-5;-5.1];               % Play around with poles here: Slow poles [-2;-2.1], Fast poles [-5;-5.1];

K_dec_x     = place(A_dec_x,B_dec_x,xpoles);
K_dec_y     = place(A_dec_y,B_dec_y,ypoles);
K_dec_z     = place(A_dec_z,B_dec_z,zpoles);
K_dec_yaw   = place(A_dec_yaw,B_dec_yaw,yawpoles);

% Compute Full-state feedback for original system
K_poleplace = [K_dec_x K_dec_z K_dec_y K_dec_yaw]*inv(Veig_nrm);
K_poleplace(abs(K_poleplace)<1e-10)=0;

%The difference between the two controllers is only in z-dynamics. We
%expect the controller with z-poles {-5;-5.1} to be stronger/faster than
%the one with z-poles {-3;-3.1}. As the z-subsystem acts on states Z and
%zdot, the only difference that we see in the K matrices are the elements
%relating to the states Z and zdot.

%Print K matrix
K_poleplace

% Generate c-code ready format for copy-paste straight into src-files rsedu_control.c
K_poleplace_string = sprintf('%E,' , K_poleplace(:));
K_poleplace_string = ['{ ' K_poleplace_string(1:end-1) ' }']

% Check where poles land
% [V,E,]=eig(A_dec_x-B_dec_x*K_dec_x);
% [V,E,]=eig(A_dec_y-B_dec_y*K_dec_y);