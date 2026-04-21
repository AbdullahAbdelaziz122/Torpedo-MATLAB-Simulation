function actuation_sfcn(block)
%% actuation_sfcn.m  —  Level-2 MATLAB S-Function: Actuation Module (M4)
%
% PURPOSE:
%   Implements the Actuation subsystem as a Simulink Level-2 S-Function.
%   Translates generalised control forces (tau_ctrl) into physical
%   actuator commands (ui) with hardware limits enforced.
%   Also generates PWM microsecond values for the HIL layer.
%
% BLOCK PORTS:
%   Inputs:
%     u1 : tau_ctrl [6×1]  — generalised forces [X Y Z K M N] from Control
%     u2 : U        [1×1]  — vehicle speed from Dynamics (for fin effectiveness)
%     u3 : n_direct [1×1]  — direct RPM command from speed controller
%                            (bypasses tau_ctrl(1), see design note below)
%
%   Outputs:
%     y1 : ui       [3×1]  — physical commands [delta_r(rad), delta_s(rad), n(RPM)]
%     y2 : pwm      [3×1]  — PWM values [servo1_µs, servo2_µs, esc_µs]
%     y3 : sat_flags[3×1]  — uint8 saturation flags per channel
%
% DESIGN NOTE — Surge/RPM channel:
%   Two control paths can feed RPM:
%     a) tau_ctrl(1) = X force → thrust inversion → RPM   (model-based)
%     b) n_direct = RPM        → direct RPM command        (speed PID output)
%   In Phase 5, the speed controller will output RPM directly (path b).
%   Path a is reserved for future model-based surge control.
%   The selector is: if n_direct > 0, use n_direct; else use tau inversion.
%   During Phase 3 testing, n_direct is set to 0 (no speed controller yet).
%
% DIRECT FEEDTHROUGH:
%   All outputs depend on current inputs → DirectFeedthrough = true for all.
%   This is correct because there are no continuous states in this block
%   (Actuation is a purely algebraic mapping, not a dynamic system).
%
% AUTHOR: AUV Simulation Project — Phase 3

setup(block);

% =========================================================================
function setup(block)

block.NumInputPorts  = 3;
block.NumOutputPorts = 3;

% u1: tau_ctrl [6×1]
block.InputPort(1).Dimensions  = 6;
block.InputPort(1).DatatypeID  = 0;
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = true;

% u2: U [scalar speed]
block.InputPort(2).Dimensions  = 1;
block.InputPort(2).DatatypeID  = 0;
block.InputPort(2).Complexity  = 'Real';
block.InputPort(2).DirectFeedthrough = true;

% u3: n_direct [scalar RPM from speed controller]
block.InputPort(3).Dimensions  = 1;
block.InputPort(3).DatatypeID  = 0;
block.InputPort(3).Complexity  = 'Real';
block.InputPort(3).DirectFeedthrough = true;

% y1: ui [3×1] physical commands
block.OutputPort(1).Dimensions = 3;
block.OutputPort(1).DatatypeID = 0;
block.OutputPort(1).Complexity = 'Real';

% y2: pwm [3×1]
block.OutputPort(2).Dimensions = 3;
block.OutputPort(2).DatatypeID = 0;
block.OutputPort(2).Complexity = 'Real';

% y3: sat_flags [3×1] uint8
block.OutputPort(3).Dimensions = 3;
block.OutputPort(3).DatatypeID = 3;  % uint8
block.OutputPort(3).Complexity = 'Real';

% No continuous states — purely algebraic
block.NumContStates = 0;

% Discrete sample time matching simulation Ts
block.SampleTimes = [-1 0];  % inherited sample time

block.SimStateCompliance = 'DefaultSimState';

block.RegBlockMethod('InitializeConditions', @InitConditions);
block.RegBlockMethod('Outputs',              @Outputs);
block.RegBlockMethod('Terminate',            @Terminate);

% =========================================================================
function InitConditions(block) %#ok<INUSD>
% No states to initialize

% =========================================================================
function Outputs(block)

tau_ctrl = block.InputPort(1).Data;   % [6×1]
U        = block.InputPort(2).Data;   % scalar
n_direct = block.InputPort(3).Data;   % scalar RPM (0 = inactive)

% Load parameters
try
    auv = evalin('base', 'auv');
catch
    error('actuation_sfcn: auv struct not found. Run auv_params.m.');
end

% Compute fin commands via force inversion
[ui_tau, sat_flags] = tau_to_ui(tau_ctrl, U, auv);

% RPM channel: prefer direct command if non-zero
if n_direct > 0
    ui_tau(3)   = n_direct;
    sat_flags(3) = uint8(0);
end

% Re-saturate after direct RPM assignment
[ui_sat, sat_flags] = saturate_ui(ui_tau, auv);

% Compute PWM
pwm = ui_to_pwm(ui_sat, auv);

% Assign outputs
block.OutputPort(1).Data = ui_sat;
block.OutputPort(2).Data = pwm;
block.OutputPort(3).Data = sat_flags;

% =========================================================================
function Terminate(block) %#ok<INUSD>

% =========================================================================
% Embedded copies of actuation_lib functions
% (duplicated here so the S-Function is self-contained;
%  actuation_lib.m remains the canonical source for testing)
% =========================================================================

function [ui, sat_flags] = tau_to_ui(tau_ctrl, U, auv)
rho   = auv.phys.rho;
U_eff = max(U, 0.3);

denom_r  = 0.5 * rho * U_eff^2 * auv.act.A_r * auv.act.CL_delta_r;
delta_r_cmd = tau_ctrl(2) / denom_r;

denom_s  = 0.5 * rho * U_eff^2 * auv.act.A_s * auv.act.CL_delta_s;
delta_s_cmd = tau_ctrl(3) / denom_s;

n_cmd = rpm_from_thrust(tau_ctrl(1), U_eff, auv);

[ui, sat_flags] = saturate_ui([delta_r_cmd; delta_s_cmd; n_cmd], auv);

function [ui_sat, sat_flags] = saturate_ui(ui_raw, auv)
ui_sat    = zeros(3,1);
sat_flags = uint8(zeros(3,1));

ui_sat(1) = max(auv.act.delta_min, min(auv.act.delta_max, ui_raw(1)));
if     ui_raw(1) > auv.act.delta_max, sat_flags(1) = uint8(1);
elseif ui_raw(1) < auv.act.delta_min, sat_flags(1) = uint8(2); end

ui_sat(2) = max(auv.act.delta_min, min(auv.act.delta_max, ui_raw(2)));
if     ui_raw(2) > auv.act.delta_max, sat_flags(2) = uint8(1);
elseif ui_raw(2) < auv.act.delta_min, sat_flags(2) = uint8(2); end

ui_sat(3) = max(auv.act.n_min, min(auv.act.n_max, ui_raw(3)));
if     ui_raw(3) > auv.act.n_max, sat_flags(3) = uint8(1);
elseif ui_raw(3) < auv.act.n_min, sat_flags(3) = uint8(2); end

function pwm = ui_to_pwm(ui_sat, auv)
pwm = zeros(3,1);
pwm(1) = auv.act.pwm_fin_neutral + (ui_sat(1)/auv.act.delta_max)*auv.act.pwm_fin_range;
pwm(2) = auv.act.pwm_fin_neutral + (ui_sat(2)/auv.act.delta_max)*auv.act.pwm_fin_range;
pwm(3) = auv.act.pwm_esc_min + (ui_sat(3)/auv.act.n_max)*(auv.act.pwm_esc_max - auv.act.pwm_esc_min);
pwm    = max(1000, min(2000, pwm));

function n_rpm = rpm_from_thrust(T_desired, U, auv)
rho=auv.phys.rho; D=auv.prop.D_prop; t=auv.prop.t_prop;
wf=auv.prop.wake_frac; KT_0=auv.prop.KT_0;
KT_max=auv.prop.KT_max; Ja_max=auv.prop.Ja_max;
Va=wf*U; n_est=1000/60;
Ja_est=min(Va/max(n_est*D,1e-6), Ja_max);
KT_est=KT_0+(KT_max-KT_0)/Ja_max*Ja_est;
denom=(1-t)*rho*D^4*KT_est;
if T_desired >= 0
    n_rps = sqrt(max(T_desired/max(denom,1e-9), 0));
else
    n_rps = 0;
end
n_rpm = max(auv.act.n_min, min(auv.act.n_max, n_rps*60));
