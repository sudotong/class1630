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
state_equil = [0; 0; -1.5; 0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ];
input_equil = [-quad.g*quad.M ;0 ;0 ;0];
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
p_ddot          = Global2Body*[0;0;quad.g] + T/quad.M*[0;0;1];
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
x_0 = state_equil
u_0 = input_equil

%% 1.2) Linearizing Full Nonlinear Simulink Model (the model from Robotics Toolbox)
%use Simulation/controllers/controller_fullstate/linearizeDrone.slx and Simulink's ControlDesign/Linear Analysis

%% 2.0) Load Full-state Feedback Controller derived from the PIDtoW-controller
%(see linearizePID2W.m)
K_pid =  [0 0 0.425862895347363 0 0 0 0 0 0.248420022285962 0 0 0;0 0 0 0.0102792311510984 0 0 0 0 0 0 0 0.00308376934532953;-0.00425177438846976 0 0 0 0.013286794963968 0 -0.00106294359711744 0 0 0 0.0017715726618624 0;0 0.00834853616902657 0 0 0 0.028788055755264 0 0.00259092501797376 0 0.00664339748198401 0 0];

% Generate c-code ready format for copy-paste straight into src-files rsedu_control.c
K_pid_ccode_string = sprintf('%E,' , K_pid(:));
K_pid_ccode_string = ['{ ' K_pid_ccode_string(1:end-1) ' }']

%% 2.1) Designing Full-state Feedback Controllers with Simplified Dynamics Model (1.1) via Pole Placement

% Find states to decouple
[V,J]   = jordan(A);
Veig_nrm = diag(1./sum(V,1))*V;
% System matrices of decoupled system
A_dec   = inv(Veig_nrm)*A*Veig_nrm;
B_dec   = inv(Veig_nrm)*B;

% Define decoupled subsystems
A_dec_x = double(A_dec(1:4,1:4));
A_dec_z = double(A_dec(5:6,5:6));
A_dec_y = double(A_dec(7:10,7:10));
A_dec_yaw = double(A_dec(11:12,11:12));

B_dec_x = double(B_dec(1:4,1:4));
B_dec_z = double(B_dec(5:6,1:4));
B_dec_y = double(B_dec(7:10,1:4));
B_dec_yaw = double(B_dec(11:12,1:4));

% Now place your own poles for the decoupled subsystems separately
K_dec_x = place(A_dec_x,B_dec_x,[-9+6i;-9-6i;-.18+1.8i;-.18-1.8i]);
K1_dec_alt = place(A_dec_z,B_dec_z,[-2;-2.1]); % controller 1
K2_dec_alt = place(A_dec_z,B_dec_z,[-5;-5.1]); % controller 2
K_dec_y = place(A_dec_y,B_dec_y,[-60;-4;-.1+2i;-.1-2i]);
K_dec_yaw = place(A_dec_yaw,B_dec_yaw,[-3,-3.1]);

% Compute Full-state feedback for 'original' system
K_poleplace_1 = [K_dec_x K1_dec_alt K_dec_y K_dec_yaw]*inv(Veig_nrm);
K_poleplace_1(abs(K_poleplace_1)<1e-10)=0;

K_poleplace_2 = [K_dec_x K2_dec_alt K_dec_y K_dec_yaw]*inv(Veig_nrm);
K_poleplace_2(abs(K_poleplace_2)<1e-10)=0;


% Generate c-code ready format for copy-paste straight into src-files rsedu_control.c
% K_poleplace_string = sprintf('%E,' , K_poleplace(:));
% K_poleplace_string = ['{ ' K_poleplace_string(1:end-1) ' }']



%% 2.2) Designing Full-state Feedback Controllers with Simplified Dynamics Model (1.1) via LQR

%CODE MISSING

K_lqr(abs(K_lqr)<(1e-10))=0;  %set small values zero
K_lqr(2,:) = K_poleplace(2,:); %LQR does not work well on yaw as LQR places much weight on yaw-rate (which is quite noisy)

% Generate c-code ready format for copy-paste to src-files.
K_lqr_ccode_string = sprintf('%E,' , K_lqr(:));
K_lqr_ccode_string = ['{ ' K_lqr_ccode_string(1:end-1) ' }']
 
%% question 4)
load('linsys1.mat')
A4 = linsys1.c*linsys1.a*inv(linsys1.c);
B4 = linsys1.c*linsys1.b;
C4 = linsys1.c*inv(linsys1.c);
D4 = 0;

%% produce graphs
close all

gx = simout.data(:,1);
gy = simout.data(:,2);
gz = simout.data(:,3);
gyaw = simout.data(:,4);
gpitch = simout.data(:,5);
groll = simout.data(:,6);
gt = simout.time;
figure
plot(gt, gx, 'b')
hold on
plot(gt, gy, 'r')
plot(gt, gz, 'g')
legend('x','y','z')
title('Positions')

figure
plot(gt, gyaw, 'b')
hold on
plot(gt, gpitch, 'r')
plot(gt, groll, 'g')
legend('yaw','pitch','roll')
title('Orientations')

