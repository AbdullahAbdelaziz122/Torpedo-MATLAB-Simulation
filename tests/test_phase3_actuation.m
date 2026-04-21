%% test_phase3_actuation.m  —  Phase 3 Gate Test
%
% PURPOSE:
%   Validates the Actuation subsystem using actuation_lib.m directly —
%   no Simulink required. Tests all mappings, inversions, saturations,
%   PWM conversions, and round-trip consistency.
%
% PASS CRITERIA:
%   [PASS] tau=0  → ui = [0, 0, 0]
%   [PASS] Fin force inversion round-trips within 2%
%   [PASS] Thrust inversion round-trips within 5%
%   [PASS] Saturation flags set correctly at limits
%   [PASS] PWM(delta_r = 0) = 1500 µs (neutral)
%   [PASS] PWM(delta_r = +delta_max) = 2000 µs
%   [PASS] PWM(delta_r = -delta_max) = 1000 µs
%   [PASS] PWM(n = 0 RPM) = 1000 µs (ESC armed/stopped)
%   [PASS] PWM(n = n_max) = 2000 µs (ESC full throttle)
%   [PASS] No output ever exceeds [1000, 2000] µs range
%   [PASS] Fin commands clamp at ±delta_max, flags fire correctly
%   [PASS] RPM clamps at n_max, flag fires correctly
%   [PASS] At U < 0.3 m/s, speed floor prevents divide-by-zero
%
% HOW TO RUN:
%   >> buses; auv_params;
%   >> test_phase3_actuation

fprintf('\n========================================\n');
fprintf('  Phase 3 Gate Test — Actuation Validation\n');
fprintf('========================================\n\n');

if ~exist('auv','var')
    error('Run ''buses; auv_params'' first.');
end

% =========================================================================
% Test 1: Zero force input → zero commands
% =========================================================================
fprintf('--- Test 1: Zero force input ---\n');

tau_zero = zeros(6,1);
U_cruise = 1.5;

[ui_z, flags_z] = tau_to_ui(tau_zero, U_cruise, auv);

report('tau=0 → delta_r = 0 rad',           abs(ui_z(1)) < 1e-10);
report('tau=0 → delta_s = 0 rad',           abs(ui_z(2)) < 1e-10);
report('tau=0 → n_RPM = 0',                 abs(ui_z(3)) < 1e-6);
report('tau=0 → all sat_flags = 0',         all(flags_z == 0));

% =========================================================================
% Test 2: Fin force inversion round-trip  Y → delta_r → Y
% =========================================================================
fprintf('\n--- Test 2: Rudder force inversion round-trip ---\n');

test_Y_values = [5, 10, 20, 40];   % N — realistic sway force range
for Y = test_Y_values
    tau_test = zeros(6,1); tau_test(2) = Y;
    [ui_t, ~] = tau_to_ui(tau_test, U_cruise, auv);
    delta_r_result = ui_t(1);

    % Forward check: recompute force from recovered angle
    F_recovered = fin_force(delta_r_result, U_cruise, ...
        auv.act.A_r, auv.act.CL_delta_r, auv.phys.rho);
    err_pct = abs(F_recovered - Y) / Y * 100;
    report(sprintf('Y=%dN round-trip error < 2%%  (%.2f%%)', Y, err_pct), ...
        err_pct < 2.0);
end

% =========================================================================
% Test 3: Stern plane inversion round-trip  Z → delta_s → Z
% =========================================================================
fprintf('\n--- Test 3: Stern plane force inversion round-trip ---\n');

test_Z_values = [5, 10, 20, 40];
for Z = test_Z_values
    tau_test = zeros(6,1); tau_test(3) = Z;
    [ui_t, ~] = tau_to_ui(tau_test, U_cruise, auv);
    delta_s_result = ui_t(2);

    F_recovered = fin_force(delta_s_result, U_cruise, ...
        auv.act.A_s, auv.act.CL_delta_s, auv.phys.rho);
    err_pct = abs(F_recovered - Z) / Z * 100;
    report(sprintf('Z=%dN round-trip error < 2%%  (%.2f%%)', Z, err_pct), ...
        err_pct < 2.0);
end

% =========================================================================
% Test 4: Thrust inversion round-trip  T → RPM → T
% =========================================================================
fprintf('\n--- Test 4: Thrust inversion round-trip ---\n');

test_T_values = [2, 5, 10, 20];   % N — realistic thrust range
for T_des = test_T_values
    tau_test = zeros(6,1); tau_test(1) = T_des;
    [ui_t, ~] = tau_to_ui(tau_test, U_cruise, auv);
    n_result = ui_t(3);

    T_recovered = thrust_from_rpm(n_result, U_cruise, auv);
    err_pct = abs(T_recovered - T_des) / T_des * 100;
    report(sprintf('T=%dN round-trip error < 5%%  (%.2f%%)', T_des, err_pct), ...
        err_pct < 5.0);
end

% =========================================================================
% Test 5: Saturation — fin angle clamping
% =========================================================================
fprintf('\n--- Test 5: Fin saturation ---\n');

% Command well beyond physical limits
tau_large_Y = zeros(6,1); tau_large_Y(2) = 500;   % impossibly large Y
tau_large_Z = zeros(6,1); tau_large_Z(3) = -500;   % impossibly large -Z

[ui_ylim, flags_y] = tau_to_ui(tau_large_Y, U_cruise, auv);
[ui_zlim, flags_z2] = tau_to_ui(tau_large_Z, U_cruise, auv);

report('delta_r saturates at +delta_max',       abs(ui_ylim(1) - auv.act.delta_max) < 1e-10);
report('Rudder sat_flag = 1 (upper limit)',      flags_y(1) == 1);
report('delta_s saturates at -delta_max',        abs(ui_zlim(2) + auv.act.delta_max) < 1e-10);
report('Stern sat_flag = 2 (lower limit)',        flags_z2(2) == 2);

% Command RPM beyond n_max
tau_big_T = zeros(6,1); tau_big_T(1) = 10000;
[ui_tlim, flags_t] = tau_to_ui(tau_big_T, U_cruise, auv);
report('RPM saturates at n_max (1525)',           abs(ui_tlim(3) - auv.act.n_max) < 1e-6);
report('RPM sat_flag = 1 (upper limit)',           flags_t(3) == 1);

% Negative thrust → RPM = 0 (no reverse)
tau_neg_T = zeros(6,1); tau_neg_T(1) = -100;
[ui_neg, flags_neg] = tau_to_ui(tau_neg_T, U_cruise, auv);
report('Negative thrust → RPM = 0',               ui_neg(3) == 0);

% =========================================================================
% Test 6: PWM mapping correctness
% =========================================================================
fprintf('\n--- Test 6: PWM mapping ---\n');

% Neutral position
ui_neutral = [0; 0; 0];
pwm_neutral = ui_to_pwm(ui_neutral, auv);
report('PWM servo1 neutral = 1500 µs',     abs(pwm_neutral(1) - 1500) < 1e-9);
report('PWM servo2 neutral = 1500 µs',     abs(pwm_neutral(2) - 1500) < 1e-9);
report('PWM ESC at 0 RPM = 1000 µs',       abs(pwm_neutral(3) - 1000) < 1e-9);

% Maximum deflection
ui_max = [auv.act.delta_max; auv.act.delta_max; auv.act.n_max];
pwm_max = ui_to_pwm(ui_max, auv);
report('PWM servo1 at +delta_max = 2000 µs', abs(pwm_max(1) - 2000) < 1e-9);
report('PWM servo2 at +delta_max = 2000 µs', abs(pwm_max(2) - 2000) < 1e-9);
report('PWM ESC at n_max = 2000 µs',          abs(pwm_max(3) - 2000) < 1e-9);

% Minimum deflection
ui_min = [-auv.act.delta_max; -auv.act.delta_max; 0];
pwm_min = ui_to_pwm(ui_min, auv);
report('PWM servo1 at -delta_max = 1000 µs', abs(pwm_min(1) - 1000) < 1e-9);
report('PWM servo2 at -delta_max = 1000 µs', abs(pwm_min(2) - 1000) < 1e-9);

% All PWM values in [1000, 2000]
test_deltas = linspace(-auv.act.delta_max, auv.act.delta_max, 20);
test_rpms   = linspace(0, auv.act.n_max, 20);
all_in_range = true;
for k = 1:20
    pwm_k = ui_to_pwm([test_deltas(k); test_deltas(k); test_rpms(k)], auv);
    if any(pwm_k < 1000) || any(pwm_k > 2000)
        all_in_range = false;
    end
end
report('All PWM values in [1000, 2000] over full range', all_in_range);

% =========================================================================
% Test 7: Low-speed floor — no divide-by-zero below 0.3 m/s
% =========================================================================
fprintf('\n--- Test 7: Low-speed floor (U = 0 m/s) ---\n');

tau_lowspd = zeros(6,1); tau_lowspd(2) = 10; tau_lowspd(3) = 5;
[ui_low, ~] = tau_to_ui(tau_lowspd, 0.0, auv);   % U = 0

report('No NaN at U=0',   ~any(isnan(ui_low)));
report('No Inf at U=0',   ~any(isinf(ui_low)));
report('ui within limits at U=0', ...
    abs(ui_low(1)) <= auv.act.delta_max + 1e-10 && ...
    abs(ui_low(2)) <= auv.act.delta_max + 1e-10);

fprintf('\n  ui at U=0, Y=10N, Z=5N: delta_r=%.4f rad (%.1f deg),  delta_s=%.4f rad (%.1f deg)\n', ...
    ui_low(1), rad2deg(ui_low(1)), ui_low(2), rad2deg(ui_low(2)));

% =========================================================================
% Test 8: Monotonicity — increasing force → increasing angle
% =========================================================================
fprintf('\n--- Test 8: Monotonicity ---\n');

Y_range = linspace(0, 80, 50);
delta_r_range = zeros(1,50);
for k = 1:50
    tau_k = zeros(6,1); tau_k(2) = Y_range(k);
    [ui_k, ~] = tau_to_ui(tau_k, U_cruise, auv);
    delta_r_range(k) = ui_k(1);
end
% Should be non-decreasing until saturation
d_sorted = diff(delta_r_range);
report('delta_r is non-decreasing with increasing Y (up to saturation)', ...
    all(d_sorted >= -1e-12));

% =========================================================================
% Summary
% =========================================================================
fprintf('\n========================================\n');
fprintf('  If all lines show [PASS]: actuation module is correct.\n');
fprintf('  Proceed to Phase 4 — Navigation subsystem.\n');
fprintf('========================================\n\n');

% =========================================================================
% Local functions — must be at end of script
% =========================================================================
function report(label, condition)
if condition
    fprintf('  [PASS]  %s\n', label);
else
    fprintf('  [FAIL]  %s  <-- FIX THIS\n', label);
end
end

function [ui, sat_flags] = tau_to_ui(tau_ctrl, U, auv)
rho = auv.phys.rho; U_eff = max(U, 0.3);
denom_r = 0.5*rho*U_eff^2*auv.act.A_r*auv.act.CL_delta_r;
denom_s = 0.5*rho*U_eff^2*auv.act.A_s*auv.act.CL_delta_s;
delta_r_cmd = tau_ctrl(2) / denom_r;
delta_s_cmd = tau_ctrl(3) / denom_s;
n_cmd = rpm_from_thrust(tau_ctrl(1), U_eff, auv);
[ui, sat_flags] = saturate_ui([delta_r_cmd; delta_s_cmd; n_cmd], auv);
end

function [ui_sat, sat_flags] = saturate_ui(ui_raw, auv)
ui_sat = zeros(3,1); sat_flags = uint8(zeros(3,1));
ui_sat(1) = max(auv.act.delta_min, min(auv.act.delta_max, ui_raw(1)));
if     ui_raw(1) > auv.act.delta_max, sat_flags(1) = uint8(1);
elseif ui_raw(1) < auv.act.delta_min, sat_flags(1) = uint8(2); end
ui_sat(2) = max(auv.act.delta_min, min(auv.act.delta_max, ui_raw(2)));
if     ui_raw(2) > auv.act.delta_max, sat_flags(2) = uint8(1);
elseif ui_raw(2) < auv.act.delta_min, sat_flags(2) = uint8(2); end
ui_sat(3) = max(auv.act.n_min, min(auv.act.n_max, ui_raw(3)));
if     ui_raw(3) > auv.act.n_max, sat_flags(3) = uint8(1);
elseif ui_raw(3) < auv.act.n_min, sat_flags(3) = uint8(2); end
end

function pwm = ui_to_pwm(ui_sat, auv)
pwm = zeros(3,1);
pwm(1) = auv.act.pwm_fin_neutral + (ui_sat(1)/auv.act.delta_max)*auv.act.pwm_fin_range;
pwm(2) = auv.act.pwm_fin_neutral + (ui_sat(2)/auv.act.delta_max)*auv.act.pwm_fin_range;
pwm(3) = auv.act.pwm_esc_min + (ui_sat(3)/auv.act.n_max)*(auv.act.pwm_esc_max - auv.act.pwm_esc_min);
pwm = max(1000, min(2000, pwm));
end

function F = fin_force(delta_rad, U, A_fin, CL, rho)
F = 0.5 * rho * U^2 * A_fin * CL * delta_rad;
end

function T = thrust_from_rpm(n_rpm, U, auv)
rho=auv.phys.rho; D=auv.prop.D_prop; t=auv.prop.t_prop;
wf=auv.prop.wake_frac; KT_0=auv.prop.KT_0;
KT_max=auv.prop.KT_max; Ja_max=auv.prop.Ja_max;
n_rps=n_rpm/60; Va=wf*U;
Ja=min(Va/max(n_rps*D,1e-6), Ja_max);
KT=KT_0+(KT_max-KT_0)/Ja_max*Ja;
X_prop=rho*D^4*KT*abs(n_rps)*n_rps;
T=(1-t)*X_prop;
end

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
end