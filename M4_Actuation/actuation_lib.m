%% actuation_lib.m  —  Actuation Model Function Library  (Rev 2)
%
% CHANGES FROM Rev 1:
%   Bug 1 — Cruciform tail factor clarified:
%            The inversion formula is:
%              delta = F_total / (0.5*rho*U^2 * A_r * CL)
%            where A_r = 2*S_fin (already the combined two-fin area from
%            auv_params). This is consistent with remus100.m's forward
%            model. The real cause of the Test 2/3 failures was Bug 3
%            (internal clamping was corrupting the recovered force before
%            the round-trip comparison). With Bug 3 fixed, Tests 2/3 pass.
%
%   Bug 2 — Iterative Ja / KT convergence in rpm_from_thrust:
%            5-step fixed-point loop instead of static 1000-RPM initial
%            guess. Eliminates the 11.57% bias at low thrust values.
%
%   Bug 3 — Single saturation boundary:
%            Removed ALL clamping from rpm_from_thrust and tau_to_ui.
%            saturate_ui is the ONLY place in the architecture that
%            applies min/max limits. Raw mathematical values (even
%            physically impossible ones like 5000 RPM) now pass through
%            freely until they reach saturate_ui.
%
% FUNCTIONS:
%   tau_to_ui        — generalised forces → raw physical commands
%   saturate_ui      — hardware limits  ← THE ONLY saturation point
%   ui_to_pwm        — physical commands → PWM microseconds
%   fin_force        — forward: fin angle → force (for testing)
%   thrust_from_rpm  — forward: RPM → thrust
%   rpm_from_thrust  — inverse: thrust → raw RPM  (no clamping)

% =========================================================================
function [ui_sat, sat_flags] = tau_to_ui(tau_ctrl, U, auv)
%% tau_to_ui — Generalised forces → saturated physical commands
%
% INPUTS:
%   tau_ctrl [6×1]  [X Y Z K M N]  generalised forces (N, N·m)
%   U        [1×1]  vehicle speed (m/s)
%   auv             parameter struct
%
% OUTPUTS:
%   ui_sat   [3×1]  [delta_r(rad), delta_s(rad), n(RPM)] — saturated
%   sat_flags[3×1]  uint8 per channel: 0=free, 1=upper, 2=lower

rho   = auv.phys.rho;
U_eff = max(U, 0.3);   % speed floor — fins are ineffective below 0.3 m/s

% Rudder: tau_ctrl(2) = Y → delta_r
% Inversion of:  Y = 0.5*rho*U^2 * A_r * CL * delta_r
% A_r = 2*S_fin (combined two-rudder area) — already in auv.act.A_r
denom_r  = 0.5 * rho * U_eff^2 * auv.act.A_r * auv.act.CL_delta_r;
delta_r  = tau_ctrl(2) / denom_r;        % raw, unbounded (rad)

% Stern plane: tau_ctrl(3) = Z → delta_s
% Inversion of:  Z = 0.5*rho*U^2 * A_s * CL * delta_s
denom_s  = 0.5 * rho * U_eff^2 * auv.act.A_s * auv.act.CL_delta_s;
delta_s  = tau_ctrl(3) / denom_s;        % raw, unbounded (rad)

% Surge: tau_ctrl(1) = X → RPM via thrust inversion (Bug 3: NO clamp here)
n_rpm    = rpm_from_thrust(tau_ctrl(1), U_eff, auv);   % raw, unbounded RPM

% saturate_ui is the single enforcement point
[ui_sat, sat_flags] = saturate_ui([delta_r; delta_s; n_rpm], auv);

end

% =========================================================================
function [ui_sat, sat_flags] = saturate_ui(ui_raw, auv)
%% saturate_ui — THE SINGLE saturation boundary in the actuation chain
%
% Receives mathematically raw (potentially out-of-range) values.
% Returns clamped values and per-channel saturation flags.
% No other function in this library may clamp actuator values.
%
% sat_flags: 0 = within limits
%            1 = clamped at upper limit
%            2 = clamped at lower limit

ui_sat    = zeros(3,1);
sat_flags = uint8(zeros(3,1));

% Channel 1: delta_r (rad)
if ui_raw(1) > auv.act.delta_max
    ui_sat(1) = auv.act.delta_max;   sat_flags(1) = uint8(1);
elseif ui_raw(1) < auv.act.delta_min
    ui_sat(1) = auv.act.delta_min;   sat_flags(1) = uint8(2);
else
    ui_sat(1) = ui_raw(1);
end

% Channel 2: delta_s (rad)
if ui_raw(2) > auv.act.delta_max
    ui_sat(2) = auv.act.delta_max;   sat_flags(2) = uint8(1);
elseif ui_raw(2) < auv.act.delta_min
    ui_sat(2) = auv.act.delta_min;   sat_flags(2) = uint8(2);
else
    ui_sat(2) = ui_raw(2);
end

% Channel 3: n_RPM
if ui_raw(3) > auv.act.n_max
    ui_sat(3) = auv.act.n_max;       sat_flags(3) = uint8(1);
elseif ui_raw(3) < auv.act.n_min
    ui_sat(3) = auv.act.n_min;       sat_flags(3) = uint8(2);
else
    ui_sat(3) = ui_raw(3);
end

end

% =========================================================================
function pwm = ui_to_pwm(ui_sat, auv)
%% ui_to_pwm — Saturated commands → PWM microseconds

pwm    = zeros(3,1);
pwm(1) = auv.act.pwm_fin_neutral + ...
         (ui_sat(1) / auv.act.delta_max) * auv.act.pwm_fin_range;
pwm(2) = auv.act.pwm_fin_neutral + ...
         (ui_sat(2) / auv.act.delta_max) * auv.act.pwm_fin_range;
pwm(3) = auv.act.pwm_esc_min + ...
         (ui_sat(3) / auv.act.n_max) * ...
         (auv.act.pwm_esc_max - auv.act.pwm_esc_min);

% Safety clamp applies only to the PWM output domain, not to actuators
pwm = max(1000, min(2000, pwm));

end

% =========================================================================
function F = fin_force(delta_rad, U, A_fin, CL, rho)
%% fin_force — Forward model: fin deflection → lift force (N)
% Used in gate tests to verify round-trip consistency.

F = 0.5 * rho * U^2 * A_fin * CL * delta_rad;

end

% =========================================================================
function T = thrust_from_rpm(n_rpm, U, auv)
%% thrust_from_rpm — Forward model: RPM → effective thrust (N)
% Matches the KT model inside remus100.m exactly.

rho    = auv.phys.rho;
D      = auv.prop.D_prop;
t      = auv.prop.t_prop;
wf     = auv.prop.wake_frac;
KT_0   = auv.prop.KT_0;
KT_max = auv.prop.KT_max;
Ja_max = auv.prop.Ja_max;

n_rps  = n_rpm / 60;
Va     = wf * U;
Ja     = min(Va / max(n_rps * D, 1e-6),  Ja_max);
KT     = KT_0 + (KT_max - KT_0) / Ja_max * Ja;
X_prop = rho * D^4 * KT * abs(n_rps) * n_rps;
T      = (1 - t) * X_prop;

end

% =========================================================================
function n_rpm = rpm_from_thrust(T_desired, U, auv)
%% rpm_from_thrust — Inverse model: desired thrust → RAW RPM
%
% Bug 2 fix: 5-step fixed-point iteration to converge Ja and KT.
%   Step 1 — Guess n_rps from T using current KT
%   Step 2 — Recompute Ja from the new n_rps and current U
%   Step 3 — Recompute KT from the new Ja
%   Step 4 — Repeat (converges in 2–3 steps across the AUV operating range)
%
% Bug 3 fix: NO clamping. Returns mathematically pure value.
%   If T_desired is physically impossible (e.g. requires 5000 RPM),
%   this function returns 5000. saturate_ui will clamp to 1525 and
%   raise sat_flag = 1. This is the correct architecture.

rho    = auv.phys.rho;
D      = auv.prop.D_prop;
t      = auv.prop.t_prop;
wf     = auv.prop.wake_frac;
KT_0   = auv.prop.KT_0;
KT_max = auv.prop.KT_max;
Ja_max = auv.prop.Ja_max;

Va = wf * U;

if T_desired <= 0
    % No reverse — return 0 RPM raw (saturate_ui will apply n_min = 0)
    n_rpm = 0;
    return
end

% Initial KT: use midpoint of range — neutral, speed-independent guess
KT_est = (KT_0 + KT_max) / 2;

n_rps = 0;   % will be set in first iteration

for iter = 1:5
    % Invert:  T = (1-t) * rho * D^4 * KT * n^2  (for n > 0)
    denom = max((1 - t) * rho * D^4 * KT_est,  1e-9);
    n_rps = sqrt(T_desired / denom);            % raw rev/s, no clamp

    % Update Ja using this n_rps estimate
    Ja_est = Va / max(n_rps * D,  1e-6);
    Ja_est = min(Ja_est, Ja_max);   % KT model only valid within [0, Ja_max]

    % Update KT for next iteration
    KT_est = KT_0 + (KT_max - KT_0) / Ja_max * Ja_est;
end

% Return raw RPM — saturate_ui is responsible for applying [n_min, n_max]
n_rpm = n_rps * 60;

end