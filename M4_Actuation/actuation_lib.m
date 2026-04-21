%% actuation_lib.m  —  Actuation Model Function Library
%
% PURPOSE:
%   Pure-MATLAB function library for the Actuation subsystem (M4).
%   All physical mappings, inversions, saturations, and PWM conversions
%   live here. No Simulink dependency — testable standalone.
%
% FUNCTIONS:
%   tau_to_ui        — generalised forces → physical commands (main mapping)
%   saturate_ui      — apply hardware limits with saturation flags
%   ui_to_pwm        — physical commands → PWM microseconds
%   fin_force        — compute actual force produced by a fin (forward model)
%   thrust_from_rpm  — compute thrust from RPM (forward model)
%   rpm_from_thrust  — compute RPM from desired thrust (inverse model)
%
% DESIGN NOTES:
%   The mapping tau_ctrl → ui is the INVERSE of what remus100.m does
%   internally. remus100 computes forces FROM fin angles; here we compute
%   fin angles FROM desired forces. This inversion is only approximate —
%   it assumes small angles and linearises the fin lift model.
%
%   The surge channel (RPM) does not go through tau_ctrl at all in the
%   current architecture. The speed controller directly commands RPM
%   via a thrust→RPM inversion. tau_ctrl(1) = X is reserved for
%   future model-based surge control.
%
%   The sway channel tau_ctrl(2) = Y → rudder angle delta_r
%   The heave channel tau_ctrl(3) = Z → stern plane angle delta_s
%   Channels 4,5,6 (K,M,N) are currently handled by their coupling
%   into the fin forces — no separate moment control allocation.
%
% USAGE:
%   Add actuation_lib to path, then call functions directly:
%   >> [ui, flags] = tau_to_ui(tau_ctrl, U, auv)
%
% AUTHOR: AUV Simulation Project — Phase 3

% =========================================================================
function [ui, sat_flags] = tau_to_ui(tau_ctrl, U, auv)
%% tau_to_ui  —  Generalised forces → physical actuator commands
%
% INPUTS:
%   tau_ctrl  [6×1]  generalised force/moment [X Y Z K M N]
%   U         [1×1]  current vehicle speed (m/s), from dynamics output
%   auv             parameter struct from auv_params.m
%
% OUTPUTS:
%   ui        [3×1]  physical commands [delta_r(rad), delta_s(rad), n(RPM)]
%   sat_flags [3×1]  uint8 saturation flags [rudder, stern, RPM]
%                    0 = unsaturated, 1 = saturated at limit
%
% CHANNEL MAPPING:
%   tau_ctrl(2) = Y  →  delta_r  (rudder, horizontal plane)
%   tau_ctrl(3) = Z  →  delta_s  (stern planes, vertical plane)
%   tau_ctrl(1) = X  →  n_RPM   (only the feedforward thrust component)
%
%   The inversion uses linearised fin lift:
%     Y = 0.5 * rho * U^2 * A_r * CL * delta_r  →  delta_r = Y / (...)
%     Z = 0.5 * rho * U^2 * A_s * CL * delta_s  →  delta_s = Z / (...)
%
%   At low speed U < U_min, a minimum speed floor prevents divide-by-zero.

rho   = auv.phys.rho;
U_eff = max(U, 0.3);   % speed floor: below 0.3 m/s fins are ineffective

% --- Rudder: Y → delta_r ---
denom_r = 0.5 * rho * U_eff^2 * auv.act.A_r * auv.act.CL_delta_r;
delta_r_cmd = tau_ctrl(2) / denom_r;

% --- Stern plane: Z → delta_s ---
denom_s = 0.5 * rho * U_eff^2 * auv.act.A_s * auv.act.CL_delta_s;
delta_s_cmd = tau_ctrl(3) / denom_s;

% --- Surge: X → RPM (via thrust inversion) ---
n_cmd = rpm_from_thrust(tau_ctrl(1), U_eff, auv);

% --- Apply saturation and record flags ---
[ui, sat_flags] = saturate_ui([delta_r_cmd; delta_s_cmd; n_cmd], auv);

end

% =========================================================================
function [ui_sat, sat_flags] = saturate_ui(ui_raw, auv)
%% saturate_ui  —  Enforce hardware limits, return saturation flags
%
% INPUTS:
%   ui_raw  [3×1]  unsaturated commands [delta_r(rad), delta_s(rad), n(RPM)]
%   auv           parameter struct
%
% OUTPUTS:
%   ui_sat   [3×1]   saturated commands (same units)
%   sat_flags[3×1]   uint8: 0=free, 1=at upper limit, 2=at lower limit

ui_sat    = zeros(3,1);
sat_flags = uint8(zeros(3,1));

% delta_r: ±delta_max
ui_sat(1)    = max(auv.act.delta_min, min(auv.act.delta_max, ui_raw(1)));
if     ui_raw(1) >  auv.act.delta_max,  sat_flags(1) = uint8(1);
elseif ui_raw(1) <  auv.act.delta_min,  sat_flags(1) = uint8(2);
end

% delta_s: ±delta_max
ui_sat(2)    = max(auv.act.delta_min, min(auv.act.delta_max, ui_raw(2)));
if     ui_raw(2) >  auv.act.delta_max,  sat_flags(2) = uint8(1);
elseif ui_raw(2) <  auv.act.delta_min,  sat_flags(2) = uint8(2);
end

% n_RPM: [n_min, n_max]
ui_sat(3)    = max(auv.act.n_min, min(auv.act.n_max, ui_raw(3)));
if     ui_raw(3) >  auv.act.n_max,  sat_flags(3) = uint8(1);
elseif ui_raw(3) <  auv.act.n_min,  sat_flags(3) = uint8(2);
end

end

% =========================================================================
function pwm = ui_to_pwm(ui_sat, auv)
%% ui_to_pwm  —  Physical commands → PWM microseconds
%
% INPUTS:
%   ui_sat  [3×1]  saturated commands [delta_r(rad), delta_s(rad), n(RPM)]
%   auv           parameter struct
%
% OUTPUTS:
%   pwm     [3×1]  PWM pulse widths in microseconds
%                  pwm(1) = servo1 (rudder),  range [1000, 2000] µs
%                  pwm(2) = servo2 (stern),   range [1000, 2000] µs
%                  pwm(3) = ESC   (thruster), range [1000, 2000] µs
%
% MAPPING:
%   Fins: neutral=1500µs, ±delta_max maps to ±500µs
%     PWM = 1500 + (delta / delta_max) * 500
%   ESC:  n=0 RPM → 1000µs, n=n_max → 2000µs
%     PWM = 1000 + (n / n_max) * 1000

pwm = zeros(3,1);

% Fin channels
pwm(1) = auv.act.pwm_fin_neutral + ...
         (ui_sat(1) / auv.act.delta_max) * auv.act.pwm_fin_range;
pwm(2) = auv.act.pwm_fin_neutral + ...
         (ui_sat(2) / auv.act.delta_max) * auv.act.pwm_fin_range;

% ESC channel
pwm(3) = auv.act.pwm_esc_min + ...
         (ui_sat(3) / auv.act.n_max) * ...
         (auv.act.pwm_esc_max - auv.act.pwm_esc_min);

% Clamp to valid PWM range (safety)
pwm = max(1000, min(2000, pwm));

end

% =========================================================================
function F_fin = fin_force(delta_rad, U, A_fin, CL, rho)
%% fin_force  —  Forward model: fin angle → lift force (N)
%
% Used in tests to verify round-trip (force → angle → force) consistency.
%
% INPUTS:
%   delta_rad  fin deflection (rad)
%   U          vehicle speed (m/s)
%   A_fin      fin planform area (m²)
%   CL         lift coefficient (-)
%   rho        fluid density (kg/m³)

F_fin = 0.5 * rho * U^2 * A_fin * CL * delta_rad;

end

% =========================================================================
function T = thrust_from_rpm(n_rpm, U, auv)
%% thrust_from_rpm  —  Forward model: RPM → thrust (N)
%
% Linear interpolation of KT between Ja=0 and Ja=Ja_max.
% Matches the model inside remus100.m.

rho    = auv.phys.rho;
D      = auv.prop.D_prop;
t      = auv.prop.t_prop;
wf     = auv.prop.wake_frac;
KT_0   = auv.prop.KT_0;
KT_max = auv.prop.KT_max;
Ja_max = auv.prop.Ja_max;

n_rps  = n_rpm / 60;          % RPM → rev/s
Va     = wf * U;               % advance speed
Ja     = Va / max(n_rps * D, 1e-6);  % advance ratio (guard /0)
Ja     = min(Ja, Ja_max);      % clip to valid range

% Linear KT model
KT = KT_0 + (KT_max - KT_0) / Ja_max * Ja;

X_prop = rho * D^4 * KT * abs(n_rps) * n_rps;
T      = (1 - t) * X_prop;    % effective thrust after deduction

end

% =========================================================================
function n_rpm = rpm_from_thrust(T_desired, U, auv)
%% rpm_from_thrust  —  Inverse model: desired thrust → RPM
%
% Inverts thrust_from_rpm using Newton iteration.
% This is the core of the surge control allocation.
%
% INPUTS:
%   T_desired  desired thrust force (N), positive = forward
%   U          current vehicle speed (m/s)
%   auv        parameter struct
%
% OUTPUT:
%   n_rpm      required propeller RPM

% Direct inversion of linear KT model (closed-form at constant Ja approx)
% T = (1-t) * rho * D^4 * KT * n|n|
% For forward thrust, n > 0:
% n^2 = T / ((1-t) * rho * D^4 * KT)
% Use KT at estimated Ja for current U

rho    = auv.phys.rho;
D      = auv.prop.D_prop;
t      = auv.prop.t_prop;
wf     = auv.prop.wake_frac;
KT_0   = auv.prop.KT_0;
KT_max = auv.prop.KT_max;
Ja_max = auv.prop.Ja_max;

% Estimate current Ja using U (not exact because we don't know n yet,
% but good enough for one-shot inversion — controller closes the loop)
Va     = wf * U;
n_est  = 1000 / 60;              % initial guess: 1000 RPM in rps
Ja_est = Va / max(n_est * D, 1e-6);
Ja_est = min(Ja_est, Ja_max);

KT_est = KT_0 + (KT_max - KT_0) / Ja_max * Ja_est;

denom  = (1 - t) * rho * D^4 * KT_est;

if T_desired >= 0
    % Forward thrust: n > 0
    n_rps = sqrt(max(T_desired / max(denom, 1e-9), 0));
else
    % Reverse command: clamp to zero (AUV has no reverse)
    n_rps = 0;
end

n_rpm = n_rps * 60;

% Final clamp to physical limits
n_rpm = max(auv.act.n_min, min(auv.act.n_max, n_rpm));

end
