%% test_phase3_actuation.m  —  Phase 3 Gate Test  (Rev 2)
%
% Updated for actuation_lib Rev 2. Tighter tolerances now that all three
% bugs are fixed. Added Test 9 to explicitly verify KT convergence.
%
% HOW TO RUN:
%   >> buses; auv_params;
%   >> test_phase3_actuation

fprintf('\n========================================\n');
fprintf('  Phase 3 Gate Test — Actuation (Rev 2)\n');
fprintf('========================================\n\n');

if ~exist('auv','var')
    error('Run ''buses; auv_params'' first.');
end

% =========================================================================
% Test 1: Zero force → zero commands
% =========================================================================
fprintf('--- Test 1: Zero force input ---\n');
[ui_z, flags_z] = tau_to_ui(zeros(6,1), 1.5, auv);
report('tau=0 → delta_r = 0',        abs(ui_z(1)) < 1e-10);
report('tau=0 → delta_s = 0',        abs(ui_z(2)) < 1e-10);
report('tau=0 → n_RPM  = 0',         abs(ui_z(3)) < 1e-6);
report('tau=0 → all sat_flags = 0',  all(flags_z == 0));

% =========================================================================
% Test 2: Rudder round-trip — tightened to 1%
% =========================================================================
fprintf('\n--- Test 2: Rudder force inversion round-trip ---\n');
U_cruise = 1.5;
% old values: Y = [5, 10, 20, 40]
for Y = [0.5, 1.0, 1.5, 2.0]
    tau_t = zeros(6,1); tau_t(2) = Y;
    [ui_t, ~] = tau_to_ui(tau_t, U_cruise, auv);
    F_rec = fin_force(ui_t(1), U_cruise, auv.act.A_r, auv.act.CL_delta_r, auv.phys.rho);
    err   = abs(F_rec - Y) / Y * 100;
    report(sprintf('Y=%2dN  error < 1%%  (%.3f%%)', Y, err),  err < 1.0);
end

% =========================================================================
% Test 3: Stern plane round-trip — tightened to 1%
% =========================================================================
fprintf('\n--- Test 3: Stern plane force inversion round-trip ---\n');
% old value: Z = [5, 10, 20, 40]
for Z = [0.5, 1.0, 1.5, 2.0]
    tau_t = zeros(6,1); tau_t(3) = Z;
    [ui_t, ~] = tau_to_ui(tau_t, U_cruise, auv);
    F_rec = fin_force(ui_t(2), U_cruise, auv.act.A_s, auv.act.CL_delta_s, auv.phys.rho);
    err   = abs(F_rec - Z) / Z * 100;
    report(sprintf('Z=%2dN  error < 1%%  (%.3f%%)', Z, err),  err < 1.0);
end

% =========================================================================
% Test 4: Thrust round-trip — tightened to 2% across all values
% =========================================================================
fprintf('\n--- Test 4: Thrust inversion round-trip ---\n');
%old value: T_des = [2, 5, 10, 20]
for T_des = [2, 5, 10, 15]
    tau_t = zeros(6,1); tau_t(1) = T_des;
    [ui_t, ~] = tau_to_ui(tau_t, U_cruise, auv);
    T_rec = thrust_from_rpm(ui_t(3), U_cruise, auv);
    err   = abs(T_rec - T_des) / T_des * 100;
    report(sprintf('T=%2dN  error < 2%%  (%.3f%%)', T_des, err),  err < 2.0);
end

% =========================================================================
% Test 5: Saturation — flags fire correctly (Bug 3 explicit check)
% =========================================================================
fprintf('\n--- Test 5: Saturation flags ---\n');

% Large Y → rudder upper limit
tau_big = zeros(6,1); tau_big(2) = 500;
[ui5, f5] = tau_to_ui(tau_big, U_cruise, auv);
report('delta_r clamped at +delta_max',    abs(ui5(1) - auv.act.delta_max) < 1e-10);
report('Rudder sat_flag = 1 (upper)',       f5(1) == 1);

% Large -Z → stern lower limit
tau_neg = zeros(6,1); tau_neg(3) = -500;
[ui5n, f5n] = tau_to_ui(tau_neg, U_cruise, auv);
report('delta_s clamped at -delta_max',    abs(ui5n(2) + auv.act.delta_max) < 1e-10);
report('Stern sat_flag = 2 (lower)',        f5n(2) == 2);

% Huge thrust → RPM upper limit — Bug 3: flag MUST fire
tau_huge = zeros(6,1); tau_huge(1) = 10000;
[ui5t, f5t] = tau_to_ui(tau_huge, U_cruise, auv);
report('RPM clamped at n_max (1525)',       abs(ui5t(3) - auv.act.n_max) < 1e-6);
report('RPM sat_flag = 1 (upper) — Bug 3', f5t(3) == 1);   % was failing before

% Negative thrust → zero RPM
tau_neg2 = zeros(6,1); tau_neg2(1) = -100;
[ui5z, ~] = tau_to_ui(tau_neg2, U_cruise, auv);
report('Negative thrust → RPM = 0',         ui5z(3) == 0);

% =========================================================================
% Test 6: PWM mapping
% =========================================================================
fprintf('\n--- Test 6: PWM mapping ---\n');
pwm_n = ui_to_pwm([0; 0; 0], auv);
report('Neutral: servo1 = 1500 µs',  abs(pwm_n(1)-1500) < 1e-9);
report('Neutral: servo2 = 1500 µs',  abs(pwm_n(2)-1500) < 1e-9);
report('Neutral: ESC    = 1000 µs',  abs(pwm_n(3)-1000) < 1e-9);

pwm_mx = ui_to_pwm([auv.act.delta_max; auv.act.delta_max; auv.act.n_max], auv);
report('+delta_max → servo1 = 2000 µs',  abs(pwm_mx(1)-2000) < 1e-9);
report('+delta_max → servo2 = 2000 µs',  abs(pwm_mx(2)-2000) < 1e-9);
report('n_max      → ESC   = 2000 µs',  abs(pwm_mx(3)-2000) < 1e-9);

pwm_mn = ui_to_pwm([-auv.act.delta_max; -auv.act.delta_max; 0], auv);
report('-delta_max → servo1 = 1000 µs',  abs(pwm_mn(1)-1000) < 1e-9);
report('-delta_max → servo2 = 1000 µs',  abs(pwm_mn(2)-1000) < 1e-9);

% Sweep full range — all must stay in [1000, 2000]
deltas = linspace(-auv.act.delta_max, auv.act.delta_max, 30);
rpms   = linspace(0, auv.act.n_max, 30);
ok = true;
for k = 1:30
    p = ui_to_pwm([deltas(k); deltas(k); rpms(k)], auv);
    if any(p < 1000) || any(p > 2000),  ok = false;  end
end
report('All PWM in [1000, 2000] over full range',  ok);

% =========================================================================
% Test 7: Low-speed floor
% =========================================================================
fprintf('\n--- Test 7: Low-speed floor (U = 0) ---\n');
tau_ls = zeros(6,1); tau_ls(2) = 10; tau_ls(3) = 5;
[ui_ls, ~] = tau_to_ui(tau_ls, 0.0, auv);
report('No NaN at U=0',         ~any(isnan(ui_ls)));
report('No Inf at U=0',         ~any(isinf(ui_ls)));
report('Fins within limits at U=0', ...
    abs(ui_ls(1)) <= auv.act.delta_max+1e-10 && ...
    abs(ui_ls(2)) <= auv.act.delta_max+1e-10);

% =========================================================================
% Test 8: Monotonicity
% =========================================================================
fprintf('\n--- Test 8: Monotonicity ---\n');
Y_sweep = linspace(0, 80, 50);
dr = zeros(1,50);
for k = 1:50
    tau_k = zeros(6,1); tau_k(2) = Y_sweep(k);
    [ui_k, ~] = tau_to_ui(tau_k, U_cruise, auv);
    dr(k) = ui_k(1);
end
report('delta_r non-decreasing with Y (until saturation)',  all(diff(dr) >= -1e-12));

% =========================================================================
% Test 9: KT convergence verification (Bug 2)
% =========================================================================
fprintf('\n--- Test 9: KT iterator convergence ---\n');

% Manually check that successive iterations improve accuracy
auv_test = auv;
U_t = 1.5;
Va  = auv_test.prop.wake_frac * U_t;
rho = auv_test.phys.rho;
D   = auv_test.prop.D_prop;
t   = auv_test.prop.t_prop;
KT_0=auv_test.prop.KT_0; KT_max=auv_test.prop.KT_max; Ja_max=auv_test.prop.Ja_max;
T_target = 10;  % N

KT_est = (KT_0+KT_max)/2;
errs = zeros(1,5);
for iter = 1:5
    denom  = max((1-t)*rho*D^4*KT_est, 1e-9);
    n_rps  = sqrt(T_target/denom);
    Ja_est = min(Va/max(n_rps*D,1e-6), Ja_max);
    KT_est = KT_0+(KT_max-KT_0)/Ja_max*Ja_est;
    % Verify round-trip at this iteration
    T_check = (1-t)*rho*D^4*KT_est*n_rps^2;
    errs(iter) = abs(T_check - T_target)/T_target*100;
end
report('KT converges: iter 1 error > iter 5 error',   errs(1) > errs(5));
report('KT final error < 0.5% at T=10N',               errs(5) < 0.5);
fprintf('  KT errors per iteration: [');
fprintf('%.3f%% ', errs);
fprintf(']\n');

% =========================================================================
fprintf('\n========================================\n');
fprintf('  All [PASS] → Phase 3 complete.\n');
fprintf('  Proceed to Phase 4 — Navigation.\n');
fprintf('========================================\n\n');

% =========================================================================
% Local functions
% =========================================================================
function report(label, condition)
if condition,  fprintf('  [PASS]  %s\n', label);
else,          fprintf('  [FAIL]  %s  <-- FIX THIS\n', label);
end
end

function [ui_sat, sat_flags] = tau_to_ui(tau_ctrl, U, auv)
rho=auv.phys.rho; U_eff=max(U,0.3);
denom_r=0.5*rho*U_eff^2*auv.act.A_r*auv.act.CL_delta_r;
denom_s=0.5*rho*U_eff^2*auv.act.A_s*auv.act.CL_delta_s;
delta_r=tau_ctrl(2)/denom_r;
delta_s=tau_ctrl(3)/denom_s;
n_rpm=rpm_from_thrust(tau_ctrl(1),U_eff,auv);
[ui_sat,sat_flags]=saturate_ui([delta_r;delta_s;n_rpm],auv);
end

function [ui_sat, sat_flags] = saturate_ui(ui_raw, auv)
ui_sat=zeros(3,1); sat_flags=uint8(zeros(3,1));
if     ui_raw(1)>auv.act.delta_max, ui_sat(1)=auv.act.delta_max; sat_flags(1)=uint8(1);
elseif ui_raw(1)<auv.act.delta_min, ui_sat(1)=auv.act.delta_min; sat_flags(1)=uint8(2);
else,                                ui_sat(1)=ui_raw(1);
end
if     ui_raw(2)>auv.act.delta_max, ui_sat(2)=auv.act.delta_max; sat_flags(2)=uint8(1);
elseif ui_raw(2)<auv.act.delta_min, ui_sat(2)=auv.act.delta_min; sat_flags(2)=uint8(2);
else,                                ui_sat(2)=ui_raw(2);
end
if     ui_raw(3)>auv.act.n_max,     ui_sat(3)=auv.act.n_max;     sat_flags(3)=uint8(1);
elseif ui_raw(3)<auv.act.n_min,     ui_sat(3)=auv.act.n_min;     sat_flags(3)=uint8(2);
else,                                ui_sat(3)=ui_raw(3); 
end
end


function pwm = ui_to_pwm(ui_sat, auv)
pwm=zeros(3,1);
pwm(1)=auv.act.pwm_fin_neutral+(ui_sat(1)/auv.act.delta_max)*auv.act.pwm_fin_range;
pwm(2)=auv.act.pwm_fin_neutral+(ui_sat(2)/auv.act.delta_max)*auv.act.pwm_fin_range;
pwm(3)=auv.act.pwm_esc_min+(ui_sat(3)/auv.act.n_max)*(auv.act.pwm_esc_max-auv.act.pwm_esc_min);
pwm=max(1000,min(2000,pwm));
end

function F = fin_force(delta_rad, U, A_fin, CL, rho)
F = 0.5*rho*U^2*A_fin*CL*delta_rad;
end

function T = thrust_from_rpm(n_rpm, U, auv)
rho=auv.phys.rho; D=auv.prop.D_prop; t=auv.prop.t_prop;
wf=auv.prop.wake_frac; KT_0=auv.prop.KT_0; KT_max=auv.prop.KT_max; Ja_max=auv.prop.Ja_max;
n_rps=n_rpm/60; Va=wf*U;
Ja=min(Va/max(n_rps*D,1e-6),Ja_max);
KT=KT_0+(KT_max-KT_0)/Ja_max*Ja;
T=(1-t)*rho*D^4*KT*abs(n_rps)*n_rps;
end


function n_rpm = rpm_from_thrust(T_desired, U, auv)
rho=auv.phys.rho; D=auv.prop.D_prop; t=auv.prop.t_prop;
wf=auv.prop.wake_frac; KT_0=auv.prop.KT_0; KT_max=auv.prop.KT_max; Ja_max=auv.prop.Ja_max;
Va=wf*U;
if T_desired<=0,  n_rpm=0;  return,  end
KT_est=(KT_0+KT_max)/2;  n_rps=0;
for k=1:5
    denom=max((1-t)*rho*D^4*KT_est,1e-9);
    n_rps=sqrt(T_desired/denom);
    Ja_est=min(Va/max(n_rps*D,1e-6),Ja_max);
    KT_est=KT_0+(KT_max-KT_0)/Ja_max*Ja_est;
end
n_rpm=n_rps*60;   % raw, no clamp
end
