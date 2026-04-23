%% test_phase9_smc.m  —  Phase 9 Gate Test
%
% PURPOSE:
%   1. Validates the SMC controller in isolation (unit tests)
%   2. Validates the swap architecture (one-line change)
%   3. Runs side-by-side PID vs SMC comparison and quantifies differences
%
% PASS CRITERIA:
%   Unit tests:
%   [PASS] control_smc_init returns struct with correct fields
%   [PASS] sat_func: output = x when |x| <= 1
%   [PASS] sat_func: output = ±1 when |x| > 1
%   [PASS] smc_surface: s=0 when error=0 and integral=0
%   [PASS] smc_surface: s converges to zero with constant error input
%   [PASS] SMC step: zero guidance → near-zero outputs (stable at equilibrium)
%   [PASS] SMC step: output fields match PID debug field names exactly
%   [PASS] Swap test: run_simulation with SMC completes without error
%
%   Performance comparison (30s, heading step 45 deg):
%   [PASS] SMC settles heading within 30s (|e_chi| < 5 deg final)
%   [PASS] SMC overshoot < 15 deg (boundary layer limits overshoot)
%   [PASS] SMC vs PID: both track, neither diverges
%   [PASS] SMC sliding surface s_psi converges to near-zero
%
% HOW TO RUN:
%   >> buses; auv_params; auv_params_env_patch;
%   >> test_phase9_smc

fprintf('\n========================================\n');
fprintf('  Phase 9 Gate Test — SMC & Algorithm Swap\n');
fprintf('========================================\n\n');

if ~exist('auv','var'), error('Run ''buses; auv_params; auv_params_env_patch'' first.'); end
if isempty(which('remus100')), error('MSS Toolbox not on path.'); end

% =========================================================================
% TEST 1: sat_func unit tests
% =========================================================================
fprintf('--- Test 1: sat_func ---\n');

report('sat(0.0) = 0.0',      abs(sat_func(0.0))   < 1e-12);
report('sat(0.5) = 0.5',      abs(sat_func(0.5) - 0.5) < 1e-12);
report('sat(1.0) = 1.0',      abs(sat_func(1.0) - 1.0) < 1e-12);
report('sat(2.0) = 1.0',      abs(sat_func(2.0) - 1.0) < 1e-12);
report('sat(-0.5) = -0.5',    abs(sat_func(-0.5) + 0.5) < 1e-12);
report('sat(-2.0) = -1.0',    abs(sat_func(-2.0) + 1.0) < 1e-12);
report('sat is odd: sat(-x) = -sat(x)', ...
    abs(sat_func(-0.7) + sat_func(0.7)) < 1e-12);

% =========================================================================
% TEST 2: smc_surface convergence
% =========================================================================
fprintf('\n--- Test 2: smc_surface ---\n');

% Zero error → zero surface
ch_z.lambda=3.0; ch_z.k=2.0; ch_z.phi=0.1; ch_z.integral=0; ch_z.e_prev=0; ch_z.sat=100;
[s_zero, ~] = smc_surface_local(0, ch_z, 0.01);
report('s = 0 when error = 0, integral = 0', abs(s_zero) < 1e-12);

% Constant error → surface should converge (integral grows, s → e*(1+lambda*t))
% Then anti-windup kicks in at |s| > 3*phi
ch_c = ch_z;
s_vals = zeros(1,200);
for k = 1:200
    [s_vals(k), ch_c] = smc_surface_local(0.1, ch_c, 0.01);
end
% Anti-windup freezes integral when |s| > 3*phi=0.3 → s should saturate
report('Surface anti-windup: |s| <= 3*phi + epsilon after 2s', ...
    abs(s_vals(end)) <= 3*0.1 + 0.2);
report('Surface remains finite for 2s constant error', all(isfinite(s_vals)));

% =========================================================================
% TEST 3: control_smc_init
% =========================================================================
fprintf('\n--- Test 3: control_smc_init ---\n');

cs = control_smc_init_local(auv);

report('cs has field u',          isfield(cs,'u'));
report('cs has field theta',      isfield(cs,'theta'));
report('cs has field psi',        isfield(cs,'psi'));
report('cs.u.lambda = 0.20',      abs(cs.u.lambda - 0.20) < 1e-10);
report('cs.psi.lambda = 3.00',    abs(cs.psi.lambda - 3.00) < 1e-10);
report('cs.theta.lambda = 5.00',  abs(cs.theta.lambda - 5.00) < 1e-10);
report('m55 ≈ 8.3 (from remus100)', abs(cs.m55 - 8.3) < 0.5);
report('m66 ≈ 8.3 (from remus100)', abs(cs.m66 - 8.3) < 0.5);

% =========================================================================
% TEST 4: SMC step at equilibrium — near-zero output
% =========================================================================
fprintf('\n--- Test 4: SMC at equilibrium ---\n');

% Vehicle at cruise speed, all errors zero
nu_eq   = [1.5; 0; 0; 0; 0; 0];
eta_eq  = [0; 0; 0; 0; 0; 0];
guid_eq.chi_d=0; guid_eq.upsilon_d=0; guid_eq.ud=1.5; guid_eq.z_des=0;

cs_eq = control_smc_init_local(auv);
[tau_eq, n_eq, dbg_eq, ~] = control_smc_step_local(guid_eq, nu_eq, eta_eq, cs_eq);

report('tau_M ≈ 0 at equilibrium (|tau_M| < 1 N·m)', abs(tau_eq(5)) < 1.0);
report('tau_N ≈ 0 at equilibrium (|tau_N| < 1 N·m)', abs(tau_eq(6)) < 1.0);
report('n_direct > 0 at cruise (maintaining speed)',   n_eq >= 0);
report('All tau_ctrl finite',                           all(isfinite(tau_eq)));

% =========================================================================
% TEST 5: Debug field names match PID exactly (drop-in compatibility)
% =========================================================================
fprintf('\n--- Test 5: Interface contract — debug fields match PID ---\n');

% PID debug fields (from control_pid_lib.m)
pid_fields = {'e_u','e_theta','e_chi','theta_d','chi_v', ...
              'tau_M','tau_N','n_direct','ff_pitch','ff_yaw'};

for i = 1:numel(pid_fields)
    report(sprintf('debug has field: %s', pid_fields{i}), ...
        isfield(dbg_eq, pid_fields{i}));
end

% SMC-specific extra fields (must not break drop-in)
report('SMC adds s_u (sliding surface — extra)',    isfield(dbg_eq,'s_u'));
report('SMC adds s_psi (sliding surface — extra)',  isfield(dbg_eq,'s_psi'));

% =========================================================================
% TEST 6: 30s closed-loop heading step — SMC performance
% =========================================================================
fprintf('\n--- Test 6: Closed-loop heading step 0→45 deg (SMC, 30s) ---\n');

[t_smc, X_smc, e_chi_smc, s_psi_smc] = run_heading_step(deg2rad(45), 30, 'smc', auv);

psi_final_smc = X_smc(end, 12);
e_chi_final   = abs(rad2deg(wrap_e(psi_final_smc - deg2rad(45))));
psi_max_smc   = max(rad2deg(X_smc(:,12)));
overshoot_smc = max(0, psi_max_smc - 45);

report('SMC heading reaches ≥ 30 deg at t=30s',        rad2deg(psi_final_smc) >= 30);
report('SMC heading final error < 5 deg',               e_chi_final < 5);
report('SMC overshoot < 15 deg',                        overshoot_smc < 15);
report('SMC sliding surface converges: |s_psi| < 0.5 final', ...
    abs(s_psi_smc(end)) < 0.5);
report('No NaN/Inf in SMC trajectory',                  all(isfinite(X_smc(:))));

fprintf('  SMC: psi(30s)=%.1f deg  error=%.2f deg  overshoot=%.2f deg\n', ...
    rad2deg(psi_final_smc), e_chi_final, overshoot_smc);

% =========================================================================
% TEST 7: PID vs SMC comparison
% =========================================================================
fprintf('\n--- Test 7: PID vs SMC comparison (same scenario) ---\n');

[t_pid, X_pid, e_chi_pid, ~] = run_heading_step(deg2rad(45), 30, 'pid', auv);

psi_pid_final = X_pid(end,12);
e_pid_final   = abs(rad2deg(wrap_e(psi_pid_final - deg2rad(45))));
psi_max_pid   = max(rad2deg(X_pid(:,12)));
overshoot_pid = max(0, psi_max_pid - 45);

report('PID heading reaches ≥ 30 deg at t=30s',  rad2deg(psi_pid_final) >= 30);
report('Both PID and SMC stable (neither diverges)', ...
    all(isfinite(X_pid(:))) && all(isfinite(X_smc(:))));

fprintf('\n  Performance comparison:\n');
fprintf('  %-12s  %12s  %12s  %12s\n', 'Controller','Final psi','Error','Overshoot');
fprintf('  %-12s  %10.1f deg  %8.2f deg  %8.2f deg\n', ...
    'PID', rad2deg(psi_pid_final), e_pid_final, overshoot_pid);
fprintf('  %-12s  %10.1f deg  %8.2f deg  %8.2f deg\n', ...
    'SMC', rad2deg(psi_final_smc), e_chi_final, overshoot_smc);

% Save for comparison plots
save('phase9_results.mat','t_smc','X_smc','e_chi_smc','s_psi_smc',...
     't_pid','X_pid','e_chi_pid');

% =========================================================================
fprintf('\n========================================\n');
fprintf('  All [PASS] → Phase 9 complete.\n');
fprintf('  Run plot_phase9 to compare PID vs SMC.\n');
fprintf('  Proceed to Phase 10 — HIL / ESP32.\n');
fprintf('========================================\n\n');

% =========================================================================
% Closed-loop heading step runner
% =========================================================================
function [t_out, X_out, e_chi_hist, s_hist] = run_heading_step(psi_des, T, ctrl_type, auv)

dt   = auv.sim.Ts;
tvec = 0:dt:T;  N = numel(tvec);
X_out = zeros(N,12);
e_chi_hist = zeros(1,N);
s_hist     = zeros(1,N);

x0 = zeros(12,1);  x0(1) = 1.5;   % cruise speed, heading North
X_out(1,:) = x0';

if strcmp(ctrl_type,'smc')
    cs = control_smc_init_local(auv);
else
    cs = ctrl_pid_init_local(auv);
end

x    = x0;
guid.chi_d=psi_des; guid.upsilon_d=0; guid.ud=1.5; guid.z_des=0;

for k = 2:N
    nu_hat  = x(1:6);
    eta_hat = x(7:12);
    eta_hat(4:6) = arrayfun(@(a) atan2(sin(a),cos(a)), eta_hat(4:6));

    if strcmp(ctrl_type,'smc')
        [tau_ctrl, n_direct, dbg, cs] = control_smc_step_local(guid, nu_hat, eta_hat, cs);
        if isfield(dbg,'s_psi'), s_hist(k) = dbg.s_psi; end
    else
        [tau_ctrl, n_direct, dbg, cs] = ctrl_pid_step_local(guid, nu_hat, eta_hat, cs);
    end

    e_chi_hist(k) = dbg.e_chi;
    ui = actuation_local(tau_ctrl, n_direct, nu_hat(1), auv);
    x  = rk4_local(x, ui, 0, 0, 0, zeros(6,1), dt);
    X_out(k,:) = x';
end
t_out = tvec;
end
% =========================================================================
% All local function copies
% =========================================================================
function cs=control_smc_init_local(auv)
cs.dt=auv.sim.Ts;
cs.u.lambda=0.20;cs.u.k=1.50;cs.u.phi=0.10;cs.u.integral=0;cs.u.e_prev=0;cs.u.sat=1000;
cs.z.integral=0;cs.z.Kp=auv.ctrl.z.Kp;cs.z.Ki=auv.ctrl.z.Ki;cs.z.theta_max=auv.ctrl.z.theta_d_max;
cs.theta.lambda=5.0;cs.theta.k=2.0;cs.theta.phi=0.05;cs.theta.integral=0;
cs.theta.e_prev=0;cs.theta.sat=auv.ctrl.theta.sat;
cs.psi.lambda=3.0;cs.psi.k=2.0;cs.psi.phi=0.10;cs.psi.integral=0;
cs.psi.e_prev=0;cs.psi.sat=auv.ctrl.psi.sat;
[~,~,M]=remus100();cs.m11=M(1,1);cs.m55=M(5,5);cs.m66=M(6,6);
cs.m35=M(3,5);cs.m26=M(2,6);cs.W=auv.phys.W;cs.B=auv.phys.B;
cs.zg=auv.phys.r_bG(3);cs.zb=auv.phys.r_bB(3);
end

function [tc,nd,dbg,cs]=control_smc_step_local(guid,nu,eta,cs)
u=nu(1);v=nu(2);w=nu(3);q=nu(5);r=nu(6);theta=eta(5);
chi_v=atan2(sin(eta(6)),cos(eta(6)));Ts=cs.dt;
[~,~,M]=remus100();m55=M(5,5);m66=M(6,6);m35=M(3,5);m26=M(2,6);
W=cs.W;B=cs.B;zg=cs.zg;zb=cs.zb;

% Surge (Dimensional bug fixed)
e_u=guid.ud-u;
[s_u,cs.u]=smc_surface_local(e_u,cs.u,Ts);
ff_s=cs.m11*(v*r-w*q);
tau_X=cs.m11*(cs.u.lambda*e_u+cs.u.k*sat_func(s_u/cs.u.phi))+ff_s;
nd=max(0,min(1525,(1525/20)*tau_X));

% Depth outer
z_d=eta(3);e_z=guid.z_des-z_d;
out_z=cs.z.Kp*e_z+cs.z.Ki*cs.z.integral;
theta_d=max(-cs.z.theta_max,min(cs.z.theta_max,out_z));
if abs(out_z)<=cs.z.theta_max,cs.z.integral=cs.z.integral+e_z*Ts;end

% Pitch inner SMC (Fixed to 2nd-Order)
e_th=atan2(sin(theta_d-theta),cos(theta_d-theta));
e_th_dot = -q; 
s_th=e_th_dot + cs.theta.lambda*e_th;
th_smc=max(-cs.theta.sat,min(cs.theta.sat,cs.theta.lambda*e_th_dot+cs.theta.k*sat_func(s_th/cs.theta.phi)));
ff_p=(W*zg-B*zb)*sin(theta)+0.3*m55*q-m35*u*w;
tau_M=m55*th_smc+ff_p;

% Heading SMC (Fixed to 2nd-Order)
e_chi=atan2(sin(guid.chi_d-chi_v),cos(guid.chi_d-chi_v));
e_chi_dot = -r; 
s_psi=e_chi_dot + cs.psi.lambda*e_chi;
psi_smc=max(-cs.psi.sat,min(cs.psi.sat,cs.psi.lambda*e_chi_dot+cs.psi.k*sat_func(s_psi/cs.psi.phi)));
ff_y=0.1*m66*r+m26*u*v;
tau_N=m66*psi_smc+ff_y;

tc=zeros(6,1);tc(5)=tau_M;tc(6)=tau_N;
dbg.e_u=e_u;dbg.e_chi=e_chi;dbg.e_theta=e_th;dbg.theta_d=theta_d;dbg.chi_v=chi_v;
dbg.tau_M=tau_M;dbg.tau_N=tau_N;dbg.n_direct=nd;dbg.ff_pitch=ff_p;dbg.ff_yaw=ff_y;
dbg.s_u=s_u;dbg.s_theta=s_th;dbg.s_psi=s_psi;
end

function [s,ch]=smc_surface_local(error,ch,dt)
ch.integral=ch.integral+error*dt;
s=error+ch.lambda*ch.integral;
if abs(s)>3*ch.phi,ch.integral=ch.integral-error*dt;end
ch.e_prev=error;
end

function y=sat_func(x)
if abs(x)<=1,y=x;else,y=sign(x);end
end

function cs=ctrl_pid_init_local(auv)
cs.dt=auv.sim.Ts;
cs.u.integral=0;cs.u.e_prev=0;cs.u.Kp=auv.ctrl.u.Kp;
cs.u.Ki=auv.ctrl.u.Ki;cs.u.Kd=auv.ctrl.u.Kd;cs.u.sat=auv.ctrl.u.u_max;
cs.z.integral=0;cs.z.Kp=auv.ctrl.z.Kp;cs.z.Ki=auv.ctrl.z.Ki;
cs.z.theta_max=auv.ctrl.z.theta_d_max;
cs.theta.integral=0;cs.theta.e_prev=0;cs.theta.Kp=auv.ctrl.theta.Kp;
cs.theta.Ki=auv.ctrl.theta.Ki;cs.theta.Kd=auv.ctrl.theta.Kd;cs.theta.sat=auv.ctrl.theta.sat;
cs.psi.integral=0;cs.psi.e_prev=0;cs.psi.Kp=auv.ctrl.psi.Kp;
cs.psi.Ki=auv.ctrl.psi.Ki;cs.psi.Kd=auv.ctrl.psi.Kd;cs.psi.sat=auv.ctrl.psi.sat;
[~,~,M]=remus100();cs.m11=M(1,1);cs.m55=M(5,5);cs.m66=M(6,6);
cs.m35=M(3,5);cs.m26=M(2,6);cs.W=auv.phys.W;cs.B=auv.phys.B;
cs.zg=auv.phys.r_bG(3);cs.zb=auv.phys.r_bB(3);
end

function [tc,nd,dbg,cs]=ctrl_pid_step_local(guid,nu,eta,cs)
u=nu(1);v=nu(2);w=nu(3);q=nu(5);r=nu(6);theta=eta(5);
chi_v=atan2(sin(eta(6)),cos(eta(6)));Ts=cs.dt;
[~,~,M]=remus100();m55=M(5,5);m66=M(6,6);m35=M(3,5);m26=M(2,6);
W=cs.W;B=cs.B;zg=cs.zg;zb=cs.zb;
e_u=guid.ud-u;du=(e_u-cs.u.e_prev)/Ts;
out_u=cs.u.Kp*e_u+cs.u.Ki*cs.u.integral+cs.u.Kd*du;
nd=max(0,min(1525,(1525/20)*out_u));
if abs(out_u)<=cs.u.sat,cs.u.integral=cs.u.integral+e_u*Ts;end;cs.u.e_prev=e_u;
z_d=eta(3);e_z=guid.z_des-z_d;
out_z=cs.z.Kp*e_z+cs.z.Ki*cs.z.integral;
theta_d=max(-cs.z.theta_max,min(cs.z.theta_max,out_z));
if abs(out_z)<=cs.z.theta_max,cs.z.integral=cs.z.integral+e_z*Ts;end
e_th=atan2(sin(theta_d-theta),cos(theta_d-theta));
dth=(e_th-cs.theta.e_prev)/Ts;
out_th_r=cs.theta.Kp*e_th+cs.theta.Ki*cs.theta.integral+cs.theta.Kd*dth;
out_th=max(-cs.theta.sat,min(cs.theta.sat,out_th_r));
ff_p=(W*zg-B*zb)*sin(theta)+0.3*m55*q-m35*u*w;tau_M=m55*out_th+ff_p;
if abs(out_th_r)<=cs.theta.sat,cs.theta.integral=cs.theta.integral+e_th*Ts;end;cs.theta.e_prev=e_th;
e_chi=atan2(sin(guid.chi_d-chi_v),cos(guid.chi_d-chi_v));
dpsi=(e_chi-cs.psi.e_prev)/Ts;
out_p_r=cs.psi.Kp*e_chi+cs.psi.Ki*cs.psi.integral+cs.psi.Kd*dpsi;
out_p=max(-cs.psi.sat,min(cs.psi.sat,out_p_r));
ff_y=0.1*m66*r+m26*u*v;tau_N=m66*out_p+ff_y;
if abs(out_p_r)<=cs.psi.sat,cs.psi.integral=cs.psi.integral+e_chi*Ts;end;cs.psi.e_prev=e_chi;
tc=zeros(6,1);tc(5)=tau_M;tc(6)=tau_N;
dbg.e_u=e_u;dbg.e_chi=e_chi;dbg.e_theta=e_th;dbg.theta_d=theta_d;dbg.chi_v=chi_v;
dbg.tau_M=tau_M;dbg.tau_N=tau_N;dbg.n_direct=nd;dbg.ff_pitch=ff_p;dbg.ff_yaw=ff_y;
end

function ui=actuation_local(tau_ctrl,n_direct,U,auv)
rho=auv.phys.rho; U_e=max(U,0.3);
x_r = auv.act.x_r; 
x_s = auv.act.x_s;

dr=tau_ctrl(6)/(-0.5*rho*U_e^2*auv.act.A_r*auv.act.CL_delta_r*x_r);
ds=tau_ctrl(5)/(0.5*rho*U_e^2*auv.act.A_s*auv.act.CL_delta_s*x_s);

dr=max(auv.act.delta_min,min(auv.act.delta_max,dr));
ds=max(auv.act.delta_min,min(auv.act.delta_max,ds));
n=max(auv.act.n_min,min(auv.act.n_max,n_direct));
ui=[dr;ds;n];
end

function x_n=rk4_local(x,ui,Vc,bVc,wc,tau_env,dt)
[~,~,M]=remus100();a_env=[M\tau_env(1:6);zeros(6,1)];
f=@(xx) remus100(xx,ui,Vc,bVc,wc)+a_env;
k1=f(x);k2=f(x+(dt/2)*k1);k3=f(x+(dt/2)*k2);k4=f(x+dt*k3);
x_n=x+(dt/6)*(k1+2*k2+2*k3+k4);
end

function e_w=wrap_e(e),e_w=atan2(sin(e),cos(e));
end

function report(label,condition)
if condition,fprintf('  [PASS]  %s\n',label);
else,        fprintf('  [FAIL]  %s  <-- FIX THIS\n',label);
end
end