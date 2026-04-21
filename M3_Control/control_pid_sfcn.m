function control_pid_sfcn(block)
%% control_pid_sfcn.m  —  Level-2 S-Function: Control Module (M3) — PID
%
% BLOCK PORTS:
%   Inputs:
%     u1 : chi_d      [1×1]  — desired course angle from Guidance (rad)
%     u2 : upsilon_d  [1×1]  — desired flight-path angle from Guidance (rad)
%     u3 : ud         [1×1]  — desired surge speed from Guidance (m/s)
%     u4 : nu_hat     [6×1]  — estimated velocities from Navigation
%     u5 : eta_hat    [6×1]  — estimated position/attitude from Navigation
%
%   Outputs:
%     y1 : tau_ctrl   [6×1]  — generalised forces → Actuation
%     y2 : n_direct   [1×1]  — direct RPM command → Actuation port 3
%     y3 : e_chi      [1×1]  — course error (rad)     } debug/logging
%     y4 : e_theta    [1×1]  — pitch error (rad)       }
%     y5 : e_u        [1×1]  — speed error (m/s)       }
%
% DISCRETE STATES:
%   6 integrator states stored as discrete states:
%     [surge_integral, z_integral, theta_integral, psi_integral,
%      surge_e_prev,   theta_e_prev,  psi_e_prev, unused]
%   Sample time = auv.sim.Ts (read from base workspace).
%
% SWAPPABILITY:
%   To replace PID with SMC: create control_smc_sfcn.m with identical
%   port layout. Change the S-Function block name in the Simulink model.
%   Nothing else changes.
%
% DIRECT FEEDTHROUGH:
%   true — control output depends on current measurement inputs.
%
% AUTHOR: AUV Simulation Project — Phase 5

setup(block);

% =========================================================================
function setup(block)

block.NumInputPorts  = 5;
block.NumOutputPorts = 5;

% u1: chi_d
block.InputPort(1).Dimensions  = 1;
block.InputPort(1).DatatypeID  = 0;
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = true;

% u2: upsilon_d
block.InputPort(2).Dimensions  = 1;
block.InputPort(2).DatatypeID  = 0;
block.InputPort(2).Complexity  = 'Real';
block.InputPort(2).DirectFeedthrough = true;

% u3: ud
block.InputPort(3).Dimensions  = 1;
block.InputPort(3).DatatypeID  = 0;
block.InputPort(3).Complexity  = 'Real';
block.InputPort(3).DirectFeedthrough = true;

% u4: nu_hat [6×1]
block.InputPort(4).Dimensions  = 6;
block.InputPort(4).DatatypeID  = 0;
block.InputPort(4).Complexity  = 'Real';
block.InputPort(4).DirectFeedthrough = true;

% u5: eta_hat [6×1]
block.InputPort(5).Dimensions  = 6;
block.InputPort(5).DatatypeID  = 0;
block.InputPort(5).Complexity  = 'Real';
block.InputPort(5).DirectFeedthrough = true;

% y1: tau_ctrl [6×1]
block.OutputPort(1).Dimensions = 6;
block.OutputPort(1).DatatypeID = 0;
block.OutputPort(1).Complexity = 'Real';

% y2: n_direct [scalar]
block.OutputPort(2).Dimensions = 1;
block.OutputPort(2).DatatypeID = 0;
block.OutputPort(2).Complexity = 'Real';

% y3-y5: debug scalars
for k = 3:5
    block.OutputPort(k).Dimensions = 1;
    block.OutputPort(k).DatatypeID = 0;
    block.OutputPort(k).Complexity = 'Real';
end

% 8 discrete states: 4 integrals + 4 previous errors (one spare)
% Layout: [u_int, z_int, theta_int, psi_int, u_eprev, theta_eprev, psi_eprev, spare]
block.NumContStates  = 0;
block.NumDworkDiscStates = 0;

% Use discrete states at simulation sample time
try
    auv = evalin('base','auv');
    Ts  = auv.sim.Ts;
catch
    Ts = 0.01;
end
block.SampleTimes    = [Ts 0];
block.NumDwork       = 8;

for k = 1:8
    block.Dwork(k).Name            = sprintf('ds%d',k);
    block.Dwork(k).Dimensions      = 1;
    block.Dwork(k).DatatypeID      = 0;   % double
    block.Dwork(k).Complexity      = 'Real';
    block.Dwork(k).UsedAsDiscState = true;
end

block.SimStateCompliance = 'DefaultSimState';

block.RegBlockMethod('InitializeConditions', @InitConditions);
block.RegBlockMethod('Outputs',              @Outputs);
block.RegBlockMethod('Update',               @Update);
block.RegBlockMethod('Terminate',            @Terminate);

% =========================================================================
function InitConditions(block)
% Zero all integrators and previous errors
for k = 1:8
    block.Dwork(k).Data = 0;
end

% =========================================================================
function Outputs(block)

chi_d     = block.InputPort(1).Data;
upsilon_d = block.InputPort(2).Data;
ud        = block.InputPort(3).Data;
nu_hat    = block.InputPort(4).Data;
eta_hat   = block.InputPort(5).Data;

try
    auv = evalin('base','auv');
catch
    error('control_pid_sfcn: auv struct not found. Run auv_params.m.');
end

% Read discrete states (integrators + prev errors)
u_int      = block.Dwork(1).Data;
z_int      = block.Dwork(2).Data;
theta_int  = block.Dwork(3).Data;
psi_int    = block.Dwork(4).Data;
u_eprev    = block.Dwork(5).Data;
theta_eprev= block.Dwork(6).Data;
psi_eprev  = block.Dwork(7).Data;

Ts = auv.sim.Ts;

% Unpack state
u=nu_hat(1); v=nu_hat(2); w=nu_hat(3);
q=nu_hat(5); r=nu_hat(6);
theta = eta_hat(5);
chi_v = atan2(sin(eta_hat(6)), cos(eta_hat(6)));  % use psi as course approx

% Get mass matrix entries (cached in auv after init — recompute if needed)
[~,~,M] = remus100();
m55 = M(5,5);  m66 = M(6,6);
m35 = M(3,5);  m26 = M(2,6);
W = auv.phys.W; B = auv.phys.B;
zg = auv.phys.r_bG(3); zb = auv.phys.r_bB(3);

% --- Surge channel ---
e_u     = ud - u;
deriv_u = (e_u - u_eprev) / Ts;
out_u_raw = auv.ctrl.u.Kp*e_u + auv.ctrl.u.Ki*u_int + auv.ctrl.u.Kd*deriv_u;
n_direct  = max(0, (1525/20) * out_u_raw);
n_direct  = min(auv.act.n_max, n_direct);

% --- Depth/pitch cascade ---
% Outer: upsilon_d gives theta_d directly from guidance
z_d   = eta_hat(3);
out_z_raw = auv.ctrl.z.Kp*(0 - z_d) + auv.ctrl.z.Ki*z_int;
theta_d   = upsilon_d + max(-auv.ctrl.z.theta_d_max, ...
                min(auv.ctrl.z.theta_d_max, out_z_raw));
theta_d   = max(-auv.ctrl.z.theta_d_max, min(auv.ctrl.z.theta_d_max, theta_d));

% Inner: theta PID
e_theta    = wrap_e(theta_d - theta);
deriv_th   = (e_theta - theta_eprev) / Ts;
out_th_raw = auv.ctrl.theta.Kp*e_theta + auv.ctrl.theta.Ki*theta_int ...
             + auv.ctrl.theta.Kd*deriv_th;
out_th     = max(-auv.ctrl.theta.sat, min(auv.ctrl.theta.sat, out_th_raw));

ff_pitch   = (W*zg - B*zb)*sin(theta) + 0.3*m55*q - m35*u*w;
tau_M      = m55 * out_th + ff_pitch;

% --- Heading channel ---
e_chi      = wrap_e(chi_d - chi_v);
deriv_psi  = (e_chi - psi_eprev) / Ts;
out_psi_raw= auv.ctrl.psi.Kp*e_chi + auv.ctrl.psi.Ki*psi_int ...
             + auv.ctrl.psi.Kd*deriv_psi;
out_psi    = max(-auv.ctrl.psi.sat, min(auv.ctrl.psi.sat, out_psi_raw));

ff_yaw     = 0.1*m66*r + m26*u*v;
tau_N      = m66 * out_psi + ff_yaw;

% Assemble tau_ctrl
tau_ctrl      = zeros(6,1);
tau_ctrl(5)   = tau_M;
tau_ctrl(6)   = tau_N;

% Outputs
block.OutputPort(1).Data = tau_ctrl;
block.OutputPort(2).Data = n_direct;
block.OutputPort(3).Data = e_chi;
block.OutputPort(4).Data = e_theta;
block.OutputPort(5).Data = e_u;

% Store current errors for derivative next step (via Update callback)
block.Dwork(5).Data = e_u;
block.Dwork(6).Data = e_theta;
block.Dwork(7).Data = e_chi;

% =========================================================================
function Update(block)
% Update integrator states with anti-windup
try
    auv = evalin('base','auv');
catch, return, end

Ts = auv.sim.Ts;

% Read errors from Dwork (set during Outputs)
e_u    = block.Dwork(5).Data;
e_th   = block.Dwork(6).Data;
e_psi  = block.Dwork(7).Data;

nu_hat  = block.InputPort(4).Data;
eta_hat = block.InputPort(5).Data;
z_d     = eta_hat(3);
ud      = block.InputPort(3).Data;
u       = nu_hat(1);

% Anti-windup: freeze integrator when at saturation limit
% Surge
u_int = block.Dwork(1).Data;
out_u = auv.ctrl.u.Kp*e_u + auv.ctrl.u.Ki*u_int;
if abs(out_u) <= auv.ctrl.u.u_max
    block.Dwork(1).Data = u_int + e_u * Ts;
end

% Depth outer
z_int = block.Dwork(2).Data;
out_z = auv.ctrl.z.Kp*(0-z_d) + auv.ctrl.z.Ki*z_int;
if abs(out_z) <= auv.ctrl.z.theta_d_max
    block.Dwork(2).Data = z_int + (0-z_d) * Ts;
end

% Pitch inner
th_int = block.Dwork(3).Data;
out_th = auv.ctrl.theta.Kp*e_th + auv.ctrl.theta.Ki*th_int;
if abs(out_th) <= auv.ctrl.theta.sat
    block.Dwork(3).Data = th_int + e_th * Ts;
end

% Heading
psi_int = block.Dwork(4).Data;
out_psi = auv.ctrl.psi.Kp*e_psi + auv.ctrl.psi.Ki*psi_int;
if abs(out_psi) <= auv.ctrl.psi.sat
    block.Dwork(4).Data = psi_int + e_psi * Ts;
end

% =========================================================================
function Terminate(block) %#ok<INUSD>

function e_w = wrap_e(e)
e_w = atan2(sin(e), cos(e));
