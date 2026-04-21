function actuation_sfcn(block)
%% actuation_sfcn.m  —  Level-2 S-Function: Actuation Module (M4)  Rev 2
%
% Embedded functions updated to match actuation_lib.m Rev 2:
%   - rpm_from_thrust: iterative Ja/KT convergence, no internal clamping
%   - tau_to_ui: no internal clamping on any channel
%   - saturate_ui: sole saturation boundary
%
% See actuation_lib.m for full documentation of each function.

setup(block);

% =========================================================================
function setup(block)

block.NumInputPorts  = 3;
block.NumOutputPorts = 3;

block.InputPort(1).Dimensions  = 6;   % tau_ctrl
block.InputPort(1).DatatypeID  = 0;
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = true;

block.InputPort(2).Dimensions  = 1;   % U (vehicle speed)
block.InputPort(2).DatatypeID  = 0;
block.InputPort(2).Complexity  = 'Real';
block.InputPort(2).DirectFeedthrough = true;

block.InputPort(3).Dimensions  = 1;   % n_direct (RPM from speed controller)
block.InputPort(3).DatatypeID  = 0;
block.InputPort(3).Complexity  = 'Real';
block.InputPort(3).DirectFeedthrough = true;

block.OutputPort(1).Dimensions = 3;   % ui [delta_r, delta_s, n_RPM]
block.OutputPort(1).DatatypeID = 0;
block.OutputPort(1).Complexity = 'Real';

block.OutputPort(2).Dimensions = 3;   % pwm [µs ×3]
block.OutputPort(2).DatatypeID = 0;
block.OutputPort(2).Complexity = 'Real';

block.OutputPort(3).Dimensions = 3;   % sat_flags uint8
block.OutputPort(3).DatatypeID = 3;   % uint8
block.OutputPort(3).Complexity = 'Real';

block.NumContStates = 0;
block.SampleTimes   = [-1 0];
block.SimStateCompliance = 'DefaultSimState';

block.RegBlockMethod('InitializeConditions', @InitConditions);
block.RegBlockMethod('Outputs',              @Outputs);
block.RegBlockMethod('Terminate',            @Terminate);

% =========================================================================
function InitConditions(block) %#ok<INUSD>

% =========================================================================
function Outputs(block)

tau_ctrl = block.InputPort(1).Data;
U        = block.InputPort(2).Data;
n_direct = block.InputPort(3).Data;

try
    auv = evalin('base', 'auv');
catch
    error('actuation_sfcn: auv struct not found. Run auv_params.m.');
end

% Compute fin + thrust commands from generalised forces
[ui_sat, sat_flags] = tau_to_ui(tau_ctrl, U, auv);

% RPM override: use direct speed-controller output if non-zero
if n_direct > 0
    % Re-run saturation on the direct RPM command
    n_raw = n_direct;
    if n_raw > auv.act.n_max
        ui_sat(3)    = auv.act.n_max;   sat_flags(3) = uint8(1);
    elseif n_raw < auv.act.n_min
        ui_sat(3)    = auv.act.n_min;   sat_flags(3) = uint8(2);
    else
        ui_sat(3)    = n_raw;           sat_flags(3) = uint8(0);
    end
end

pwm = ui_to_pwm(ui_sat, auv);

block.OutputPort(1).Data = ui_sat;
block.OutputPort(2).Data = pwm;
block.OutputPort(3).Data = sat_flags;

% =========================================================================
function Terminate(block) %#ok<INUSD>

% =========================================================================
% Embedded function copies — canonical source is actuation_lib.m Rev 2
% =========================================================================

function [ui_sat, sat_flags] = tau_to_ui(tau_ctrl, U, auv)
rho   = auv.phys.rho;
U_eff = max(U, 0.3);
denom_r = 0.5*rho*U_eff^2*auv.act.A_r*auv.act.CL_delta_r;
denom_s = 0.5*rho*U_eff^2*auv.act.A_s*auv.act.CL_delta_s;
delta_r  = tau_ctrl(2) / denom_r;
delta_s  = tau_ctrl(3) / denom_s;
n_rpm    = rpm_from_thrust(tau_ctrl(1), U_eff, auv);  % raw, no clamp
[ui_sat, sat_flags] = saturate_ui([delta_r; delta_s; n_rpm], auv);

function [ui_sat, sat_flags] = saturate_ui(ui_raw, auv)
ui_sat = zeros(3,1);  sat_flags = uint8(zeros(3,1));
if     ui_raw(1) >  auv.act.delta_max, ui_sat(1)=auv.act.delta_max; sat_flags(1)=uint8(1);
elseif ui_raw(1) <  auv.act.delta_min, ui_sat(1)=auv.act.delta_min; sat_flags(1)=uint8(2);
else,                                   ui_sat(1)=ui_raw(1); end
if     ui_raw(2) >  auv.act.delta_max, ui_sat(2)=auv.act.delta_max; sat_flags(2)=uint8(1);
elseif ui_raw(2) <  auv.act.delta_min, ui_sat(2)=auv.act.delta_min; sat_flags(2)=uint8(2);
else,                                   ui_sat(2)=ui_raw(2); end
if     ui_raw(3) >  auv.act.n_max,     ui_sat(3)=auv.act.n_max;     sat_flags(3)=uint8(1);
elseif ui_raw(3) <  auv.act.n_min,     ui_sat(3)=auv.act.n_min;     sat_flags(3)=uint8(2);
else,                                   ui_sat(3)=ui_raw(3); end

function pwm = ui_to_pwm(ui_sat, auv)
pwm    = zeros(3,1);
pwm(1) = auv.act.pwm_fin_neutral + (ui_sat(1)/auv.act.delta_max)*auv.act.pwm_fin_range;
pwm(2) = auv.act.pwm_fin_neutral + (ui_sat(2)/auv.act.delta_max)*auv.act.pwm_fin_range;
pwm(3) = auv.act.pwm_esc_min + (ui_sat(3)/auv.act.n_max)*(auv.act.pwm_esc_max-auv.act.pwm_esc_min);
pwm    = max(1000, min(2000, pwm));

function n_rpm = rpm_from_thrust(T_desired, U, auv)
% Iterative fixed-point, no clamping — matches actuation_lib Rev 2
rho=auv.phys.rho; D=auv.prop.D_prop; t=auv.prop.t_prop;
wf=auv.prop.wake_frac; KT_0=auv.prop.KT_0;
KT_max=auv.prop.KT_max; Ja_max=auv.prop.Ja_max;
Va=wf*U;
if T_desired <= 0,  n_rpm=0;  return,  end
KT_est = (KT_0+KT_max)/2;
n_rps  = 0;
for k=1:5
    denom  = max((1-t)*rho*D^4*KT_est, 1e-9);
    n_rps  = sqrt(T_desired/denom);
    Ja_est = min(Va/max(n_rps*D,1e-6), Ja_max);
    KT_est = KT_0+(KT_max-KT_0)/Ja_max*Ja_est;
end
n_rpm = n_rps*60;   % raw, no clamp