%% buses.m  —  AUV Simulation Bus Definitions
%
% PURPOSE:
%   Defines all Simulink Bus Objects for the AUV simulation.
%   Run this script before opening any Simulink model.
%   Every inter-module signal must be typed with one of these buses.
%
% USAGE:
%   >> buses          (run from MATLAB command window or startup.m)
%
% BUS INVENTORY:
%   StateBus        — 12-element full vehicle state [nu; eta]
%   NuBus           — 6-element body velocity sub-bus
%   EtaBus          — 6-element NED position/attitude sub-bus
%   GuidanceBus     — 3-element guidance references from LOS
%   ControlBus      — 6-element generalised forces + debug sub-bus
%   CtrlDebugBus    — per-channel PID internals (errors, integrals)
%   ActuationBus    — physical actuator commands + PWM values
%   EnvBus          — environmental force/moment disturbances
%   NavBus          — navigation estimates (mirrors StateBus structure)
%
% CONVENTION (remus100.m ordering — velocities first):
%   x(1:6)  = nu  = [u, v, w, p, q, r]      body-frame velocities
%   x(7:12) = eta = [x_n, y_e, z_d, phi, theta, psi]  NED position + RPY
%
% Author: Generated for AUV Simulation Project
% Phase:  1 — State Representation

clear_buses_if_exist = @(names) cellfun(@(n) ...
    evalin('base', sprintf('if exist(''%s'',''var''); clear %s; end', n, n)), names);

% =========================================================================
% 1.  NuBus  —  body-frame velocity vector  [6×1]
% =========================================================================
% Sub-bus used inside StateBus and NavBus. Keeping it separate lets you
% log or tap just velocities without carrying the full state.
clear elems
elems(1) = Simulink.BusElement;
elems(1).Name = 'u';       elems(1).Dimensions = 1;
elems(1).DataType = 'double'; elems(1).Description = 'Surge velocity (m/s)';

elems(2) = Simulink.BusElement;
elems(2).Name = 'v';       elems(2).Dimensions = 1;
elems(2).DataType = 'double'; elems(2).Description = 'Sway velocity (m/s)';

elems(3) = Simulink.BusElement;
elems(3).Name = 'w';       elems(3).Dimensions = 1;
elems(3).DataType = 'double'; elems(3).Description = 'Heave velocity (m/s)';

elems(4) = Simulink.BusElement;
elems(4).Name = 'p';       elems(4).Dimensions = 1;
elems(4).DataType = 'double'; elems(4).Description = 'Roll rate (rad/s)';

elems(5) = Simulink.BusElement;
elems(5).Name = 'q';       elems(5).Dimensions = 1;
elems(5).DataType = 'double'; elems(5).Description = 'Pitch rate (rad/s)';

elems(6) = Simulink.BusElement;
elems(6).Name = 'r';       elems(6).Dimensions = 1;
elems(6).DataType = 'double'; elems(6).Description = 'Yaw rate (rad/s)';

NuBus = Simulink.Bus;
NuBus.Description = 'Body-frame velocity vector (6×1)';
NuBus.Elements = elems;
assignin('base', 'NuBus', NuBus);

% =========================================================================
% 2.  EtaBus  —  NED position + Euler angles  [6×1]
% =========================================================================
clear elems
elems(1) = Simulink.BusElement;
elems(1).Name = 'x_n';    elems(1).Dimensions = 1;
elems(1).DataType = 'double'; elems(1).Description = 'North position (m)';

elems(2) = Simulink.BusElement;
elems(2).Name = 'y_e';    elems(2).Dimensions = 1;
elems(2).DataType = 'double'; elems(2).Description = 'East position (m)';

elems(3) = Simulink.BusElement;
elems(3).Name = 'z_d';    elems(3).Dimensions = 1;
elems(3).DataType = 'double'; elems(3).Description = 'Down position, depth (m)';

elems(4) = Simulink.BusElement;
elems(4).Name = 'phi';    elems(4).Dimensions = 1;
elems(4).DataType = 'double'; elems(4).Description = 'Roll angle (rad)';

elems(5) = Simulink.BusElement;
elems(5).Name = 'theta';  elems(5).Dimensions = 1;
elems(5).DataType = 'double'; elems(5).Description = 'Pitch angle (rad)';

elems(6) = Simulink.BusElement;
elems(6).Name = 'psi';    elems(6).Dimensions = 1;
elems(6).DataType = 'double'; elems(6).Description = 'Yaw angle (rad)';

EtaBus = Simulink.Bus;
EtaBus.Description = 'NED position and Euler angles (6×1)';
EtaBus.Elements = elems;
assignin('base', 'EtaBus', EtaBus);

% =========================================================================
% 3.  StateBus  —  full 12-state vehicle state
% =========================================================================
% Structured as two named sub-buses rather than a flat 12-vector.
% This allows any module to extract just nu or just eta cleanly.
clear elems
elems(1) = Simulink.BusElement;
elems(1).Name = 'nu';
elems(1).DataType = 'Bus: NuBus';
elems(1).Description = 'Body-frame velocities [u v w p q r]';

elems(2) = Simulink.BusElement;
elems(2).Name = 'eta';
elems(2).DataType = 'Bus: EtaBus';
elems(2).Description = 'NED position + Euler angles [x y z phi theta psi]';

StateBus = Simulink.Bus;
StateBus.Description = 'Full 12-state AUV state vector, structured as nu + eta sub-buses';
StateBus.Elements = elems;
assignin('base', 'StateBus', StateBus);

% =========================================================================
% 4.  GuidanceBus  —  outputs from Guidance (M2) → Control (M3)
% =========================================================================
% This is the INTERFACE CONTRACT between Guidance and Control.
% Any guidance algorithm (LOS, ILOS, pure-pursuit...) must output this bus.
% Any control algorithm (PID, SMC, HOSMC...) must accept this bus as input.
clear elems
elems(1) = Simulink.BusElement;
elems(1).Name = 'chi_d';
elems(1).Dimensions = 1; elems(1).DataType = 'double';
elems(1).Description = 'Desired course angle (rad) — horizontal plane heading to path';

elems(2) = Simulink.BusElement;
elems(2).Name = 'upsilon_d';
elems(2).Dimensions = 1; elems(2).DataType = 'double';
elems(2).Description = 'Desired flight-path angle (rad) — vertical plane angle to path';

elems(3) = Simulink.BusElement;
elems(3).Name = 'ud';
elems(3).Dimensions = 1; elems(3).DataType = 'double';
elems(3).Description = 'Desired surge speed (m/s) — speed along path tangent';

GuidanceBus = Simulink.Bus;
GuidanceBus.Description = 'Guidance outputs: desired course, flight-path angle, surge speed';
GuidanceBus.Elements = elems;
assignin('base', 'GuidanceBus', GuidanceBus);

% =========================================================================
% 5.  CtrlDebugBus  —  per-channel PID / controller internals
% =========================================================================
% Logged for post-analysis and real-time tuning. Never used as a control
% input by any other module — purely observability.
clear elems
elems(1) = Simulink.BusElement;
elems(1).Name = 'e_chi';
elems(1).Dimensions = 1; elems(1).DataType = 'double';
elems(1).Description = 'Course angle error (rad)';

elems(2) = Simulink.BusElement;
elems(2).Name = 'e_upsilon';
elems(2).Dimensions = 1; elems(2).DataType = 'double';
elems(2).Description = 'Flight-path angle error (rad)';

elems(3) = Simulink.BusElement;
elems(3).Name = 'e_u';
elems(3).Dimensions = 1; elems(3).DataType = 'double';
elems(3).Description = 'Surge speed error (m/s)';

elems(4) = Simulink.BusElement;
elems(4).Name = 'integral_chi';
elems(4).Dimensions = 1; elems(4).DataType = 'double';
elems(4).Description = 'Integrated course error (rad·s)';

elems(5) = Simulink.BusElement;
elems(5).Name = 'integral_upsilon';
elems(5).Dimensions = 1; elems(5).DataType = 'double';
elems(5).Description = 'Integrated flight-path error (rad·s)';

elems(6) = Simulink.BusElement;
elems(6).Name = 'integral_u';
elems(6).Dimensions = 1; elems(6).DataType = 'double';
elems(6).Description = 'Integrated speed error (m·s)';

elems(7) = Simulink.BusElement;
elems(7).Name = 'ctrl_id';
elems(7).Dimensions = 4; elems(7).DataType = 'uint8';
elems(7).Description = 'Algorithm tag: PID=1, SMC=2, HOSMC=3, AFRTSMC=4';

CtrlDebugBus = Simulink.Bus;
CtrlDebugBus.Description = 'Controller debug/observability bus — not a control input';
CtrlDebugBus.Elements = elems;
assignin('base', 'CtrlDebugBus', CtrlDebugBus);

% =========================================================================
% 6.  ControlBus  —  outputs from Control (M3) → Actuation (M4)
% =========================================================================
% tau_ctrl is the 6-DOF generalised force/moment vector.
% The controller never specifies actuator angles — it commands forces.
% Actuation translates forces to physical commands.
clear elems
elems(1) = Simulink.BusElement;
elems(1).Name = 'tau_ctrl';
elems(1).Dimensions = 6; elems(1).DataType = 'double';
elems(1).Description = '[X Y Z K M N] generalised force/moment (N, N·m)';

elems(2) = Simulink.BusElement;
elems(2).Name = 'debug';
elems(2).DataType = 'Bus: CtrlDebugBus';
elems(2).Description = 'Controller debug signals';

ControlBus = Simulink.Bus;
ControlBus.Description = 'Control module outputs: generalised forces + debug';
ControlBus.Elements = elems;
assignin('base', 'ControlBus', ControlBus);

% =========================================================================
% 7.  ActuationBus  —  outputs from Actuation (M4) → Dynamics (M5) + HIL
% =========================================================================
clear elems
elems(1) = Simulink.BusElement;
elems(1).Name = 'delta_r';
elems(1).Dimensions = 1; elems(1).DataType = 'double';
elems(1).Description = 'Rudder angle (rad), saturated to ±20 deg';

elems(2) = Simulink.BusElement;
elems(2).Name = 'delta_s';
elems(2).Dimensions = 1; elems(2).DataType = 'double';
elems(2).Description = 'Stern plane angle (rad), saturated to ±20 deg';

elems(3) = Simulink.BusElement;
elems(3).Name = 'n_rpm';
elems(3).Dimensions = 1; elems(3).DataType = 'double';
elems(3).Description = 'Propeller speed (RPM), saturated to [0, 1525]';

elems(4) = Simulink.BusElement;
elems(4).Name = 'pwm_servo1';
elems(4).Dimensions = 1; elems(4).DataType = 'double';
elems(4).Description = 'Servo 1 PWM (µs), range [1000, 2000]';

elems(5) = Simulink.BusElement;
elems(5).Name = 'pwm_servo2';
elems(5).Dimensions = 1; elems(5).DataType = 'double';
elems(5).Description = 'Servo 2 PWM (µs), range [1000, 2000]';

elems(6) = Simulink.BusElement;
elems(6).Name = 'pwm_esc';
elems(6).Dimensions = 1; elems(6).DataType = 'double';
elems(6).Description = 'ESC PWM (µs), range [1000, 2000]';

elems(7) = Simulink.BusElement;
elems(7).Name = 'sat_flags';
elems(7).Dimensions = 3; elems(7).DataType = 'uint8';
elems(7).Description = 'Saturation flags [rudder, stern, RPM] — 0=free, 1=saturated';

ActuationBus = Simulink.Bus;
ActuationBus.Description = 'Actuation outputs: physical commands + PWM + saturation flags';
ActuationBus.Elements = elems;
assignin('base', 'ActuationBus', ActuationBus);

% =========================================================================
% 8.  EnvBus  —  outputs from Environment (M1) → Dynamics (M5)
% =========================================================================
clear elems
elems(1) = Simulink.BusElement;
elems(1).Name = 'tau_env';
elems(1).Dimensions = 6; elems(1).DataType = 'double';
elems(1).Description = 'Environmental generalised force/moment (N, N·m)';

elems(2) = Simulink.BusElement;
elems(2).Name = 'Vc';
elems(2).Dimensions = 1; elems(2).DataType = 'double';
elems(2).Description = 'Current speed magnitude (m/s)';

elems(3) = Simulink.BusElement;
elems(3).Name = 'betaVc';
elems(3).Dimensions = 1; elems(3).DataType = 'double';
elems(3).Description = 'Current direction in NED (rad), 0=North';

elems(4) = Simulink.BusElement;
elems(4).Name = 'w_c';
elems(4).Dimensions = 1; elems(4).DataType = 'double';
elems(4).Description = 'Vertical current speed (m/s), positive = downward';

EnvBus = Simulink.Bus;
EnvBus.Description = 'Environment outputs: disturbance forces + ocean current parameters';
EnvBus.Elements = elems;
assignin('base', 'EnvBus', EnvBus);

% =========================================================================
% 9.  NavBus  —  outputs from Navigation (M6) → Control (M3) + Guidance (M2)
% =========================================================================
% Mirrors StateBus structure but uses _hat suffix to distinguish estimates
% from truth. In Phase 4 pass-through: eta_hat = eta_true, nu_hat = nu_true.
clear elems
elems(1) = Simulink.BusElement;
elems(1).Name = 'nu_hat';
elems(1).DataType = 'Bus: NuBus';
elems(1).Description = 'Estimated body velocities [u v w p q r]';

elems(2) = Simulink.BusElement;
elems(2).Name = 'eta_hat';
elems(2).DataType = 'Bus: EtaBus';
elems(2).Description = 'Estimated NED position + Euler angles';

elems(3) = Simulink.BusElement;
elems(3).Name = 'speed';
elems(3).Dimensions = 1; elems(3).DataType = 'double';
elems(3).Description = 'Estimated vehicle speed |nu| (m/s)';

NavBus = Simulink.Bus;
NavBus.Description = 'Navigation estimates: velocity + position + speed scalar';
NavBus.Elements = elems;
assignin('base', 'NavBus', NavBus);

% =========================================================================
% Report
% =========================================================================
fprintf('\n=== AUV Bus Definitions Loaded ===\n');
fprintf('  NuBus          : body velocities [u v w p q r]\n');
fprintf('  EtaBus         : NED position + RPY [x y z phi theta psi]\n');
fprintf('  StateBus       : full state = NuBus + EtaBus\n');
fprintf('  GuidanceBus    : [chi_d, upsilon_d, ud]\n');
fprintf('  CtrlDebugBus   : errors, integrals, ctrl_id\n');
fprintf('  ControlBus     : [tau_ctrl(6x1)] + CtrlDebugBus\n');
fprintf('  ActuationBus   : [delta_r, delta_s, n_rpm, pwm×3, sat_flags]\n');
fprintf('  EnvBus         : [tau_env(6x1), Vc, betaVc, w_c]\n');
fprintf('  NavBus         : NuBus_hat + EtaBus_hat + speed\n');
fprintf('\nAll buses assigned to base workspace.\n');
fprintf('Run ''buses'' at the start of every session.\n\n');
