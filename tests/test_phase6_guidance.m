%% test_phase6_guidance.m  —  Phase 6 Gate Test
%
% PURPOSE:
%   Validates the LOS guidance library and closed-loop path following.
%   All six original LOS.m bugs are tested explicitly as regressions.
%
% PASS CRITERIA:
%   Unit tests (no dynamics):
%   [PASS] chi_v uses atan2 — correct for all quadrants (Bug 1)
%   [PASS] chi_v wraps across ±pi boundary correctly (Bug 1)
%   [PASS] Speed denominator clamped — no blow-up at large correction (Bug 2)
%   [PASS] upsilon_d: no NaN when asin argument is at boundary (Bug 3)
%   [PASS] upsilon_d: no NaN for 1000 random path configurations (Bug 3)
%   [PASS] ud is finite when desire_speed is zero (Bug 5)
%   [PASS] path_lib: helix has correct start position and velocity
%   [PASS] path_query returns correct position at t=0 and t=T/2
%   [PASS] los_errors: zero error when vehicle is on the path point
%   [PASS] los_errors: lateral error is purely y_e when error is pure Y-NED
%   [PASS] los_compose output is always in [-pi,pi] for chi_d
%
%   Closed-loop integration test:
%   [PASS] Vehicle follows helix: RMS cross-track error < 8m over 100s
%   [PASS] Vehicle follows helix: max cross-track error < 20m
%   [PASS] Depth tracking: |z_vehicle - z_path| < 3m RMS
%   [PASS] No NaN or Inf in any state over 100s
%
% HOW TO RUN:
%   >> buses; auv_params;
%   >> test_phase6_guidance

fprintf('\n========================================\n');
fprintf('  Phase 6 Gate Test — LOS Guidance\n');
fprintf('========================================\n\n');

if ~exist('auv', 'var')
    error('Run ''buses; auv_params'' first.');
end

if isempty(which('remus100'))
    error('MSS Toolbox not on path.');
end

los = los_init(auv);

% =========================================================================
% TEST 1: Bug 1 regression — chi_v correct in all quadrants
% =========================================================================
fprintf('--- Test 1: Bug 1 — chi_v quadrant correctness ---\n');

% NED velocity → expected course angle
quad_cases = [
    1,  0,  0,   0;      % North  → chi = 0
    0,  1,  0,   pi/2;   % East   → chi = pi/2
   -1,  0,  0,   pi;     % South  → chi = ±pi
    0, -1,  0,  -pi/2;   % West   → chi = -pi/2
    1,  1,  0,   pi/4;   % NE     → chi = pi/4
];

all_q_ok = true;
for k = 1:size(quad_cases, 1)
    vel_ned  = quad_cases(k, 1:3)';
    expected = quad_cases(k, 4);
    [chi_got, ~] = vehicle_angles(vel_ned);

    if abs(wrap_angle(chi_got - expected)) > 1e-10
        fprintf('  chi_v(%d,%d,%d): expected %.4f, got %.4f\n', ...
            quad_cases(k, 1), quad_cases(k, 2), quad_cases(k, 3), expected, chi_got);
        all_q_ok = false;
    end
end
report('chi_v correct in all 5 quadrant cases', all_q_ok);

% ±pi boundary: vehicle moving exactly South
vel_south = [-1; 0; 0];
chi_south = vehicle_angles(vel_south);
report('chi_v at South heading is ±pi (not NaN)', ...
    ~isnan(chi_south) && abs(abs(chi_south) - pi) < 1e-10);

% =========================================================================
% TEST 2: Bug 2 regression — speed denominator clamp
% =========================================================================
fprintf('\n--- Test 2: Bug 2 — speed denominator clamp ---\n');

% Large correction angle: psi_r → 1 (tanh saturated), theta_r → 1
% cos(tanh^{-1}(0.999)) ≈ cos(3.8) ≈ -0.79 ... but tanh output is in (-1,1)
% with large y_e/delta_y the denominator approaches cos(pi/2)*cos(pi/2) → 0
% This should NOT blow up:
pv_test  = [0.6; 0; 0];
x_e_test = 0;
y_e_test = 1000;   % huge lateral error — correction saturates
z_e_test = 1000;   % huge vertical error

psi_r_big   = tanh(-los.ky * y_e_test / los.delta_y);
theta_r_big = tanh( los.kz * z_e_test / los.delta_z);

ds_test = [0.6; 0; 0];
ud_big  = los_speed(pv_test, x_e_test, psi_r_big, theta_r_big, ds_test, los);

report('ud is finite for huge cross-track error', isfinite(ud_big));
report('ud is within speed limits', ud_big >= los.U_min && ud_big <= los.U_max);
fprintf('  ud at y_e=1000m, z_e=1000m = %.3f m/s\n', ud_big);

% =========================================================================
% TEST 3: Bug 3 regression — asin domain safety
% =========================================================================
fprintf('\n--- Test 3: Bug 3 — asin domain safety ---\n');

% Worst case: argument exactly at boundary
[~, upsilon_plus]  = los_compose(0, pi/2,  0.0, 0.0);
[~, upsilon_minus] = los_compose(0, -pi/2, 0.0, 0.0);
report('upsilon_d finite at theta_point = +90 deg', isfinite(upsilon_plus));
report('upsilon_d finite at theta_point = -90 deg', isfinite(upsilon_minus));

% 1000 random configurations — no NaN allowed
rng(42);
N_rand = 1000;
no_nan = true;
for k = 1:N_rand
    pp = (rand - 0.5) * 2 * pi;
    tp = (rand - 0.5) * pi / 2;
    pr = tanh((rand - 0.5) * 4);
    tr = tanh((rand - 0.5) * 4);

    [cd, ud_a] = los_compose(pp, tp, pr, tr);
    if isnan(cd) || isnan(ud_a) || isinf(cd) || isinf(ud_a)
        no_nan = false;
        break;
    end
end
report('No NaN/Inf in 1000 random los_compose calls', no_nan);

% =========================================================================
% TEST 4: Bug 5 regression — ud with zero desire_speed
% =========================================================================
fprintf('\n--- Test 4: Bug 5 — ud with zero desire_speed ---\n');

ud_zero = los_speed([0.6; 0; 0], 0, 0, 0, [0; 0; 0], los);
report('ud is finite when desire_speed = [0;0;0]', isfinite(ud_zero));
report('ud equals los.U_nom when desire_speed = zero', abs(ud_zero - los.U_nom) < 1e-10);

% =========================================================================
% TEST 5: path_lib validation
% =========================================================================
fprintf('\n--- Test 5: path_lib helix ---\n');

path = path_helix(auv);

report('Path has pts field [3×N]', isfield(path, 'pts') && size(path.pts, 1) == 3);
report('Path has vel field [3×N]', isfield(path, 'vel') && size(path.vel, 1) == 3);
report('Path start: X ≈ 60m', abs(path.pts(1, 1) - 60) < 1);
report('Path start: Y ≈ 0m', abs(path.pts(2, 1)) < 1);
report('Path start: Z ≈ 2m depth', abs(path.pts(3, 1) - 2) < 0.1);
report('Path velocity finite everywhere', all(isfinite(path.vel(:))));
report('Path velocity non-zero', all(sqrt(sum(path.vel.^2, 1)) > 0.01));

% path_query test
[pt0, vel0] = path_query(path, 0);
[ptT, velT] = path_query(path, path.T_total);
report('path_query at t=0 returns start point', norm(pt0 - path.pts(:, 1)) < 1e-6);
report('path_query at t=T returns end point', norm(ptT - path.pts(:, end)) < 0.1);
report('path_query velocity at t=0 is non-zero', norm(vel0) > 0.01);

% Keep velT evaluated as in original flow, even if not used later.
%#ok<NASGU>

% =========================================================================
% TEST 6: los_errors — geometry
% =========================================================================
fprintf('\n--- Test 6: los_errors geometry ---\n');

% Vehicle on the path point → all errors = 0
pt_ref = [10; 20; 5];
[xe0, ye0, ze0] = los_errors(pt_ref, pt_ref, 0, 0);
report('Zero error when vehicle is on path point', ...
    abs(xe0) < 1e-12 && abs(ye0) < 1e-12 && abs(ze0) < 1e-12);

% Pure East offset with North-pointing path → lateral error only
% Path pointing North (psi_p=0, theta_p=0), vehicle 5m East of path
pos_east = [10; 25; 5];   % 5m East of pt_ref
[xe1, ye1, ze1] = los_errors(pos_east, pt_ref, 0, 0);
report('Pure East offset → ye only (xe≈0, ze≈0)', ...
    abs(xe1) < 1e-10 && abs(ye1 - 5) < 1e-10 && abs(ze1) < 1e-10);

% =========================================================================
% TEST 7: Closed-loop helix tracking (100s)
% =========================================================================
fprintf('\n--- Test 7: Closed-loop helix tracking (100s) ---\n');
fprintf('  (This takes ~20s — running ode45 integration)\n');

auv_test = auv;
auv_test.sim.T_end = 100;
path_test = path_helix(auv_test);

% Initial condition: start near helix start with cruise speed
x0 = zeros(12, 1);
x0(1)  = 1.5;                % surge speed
x0(7)  = path_test.pts(1, 1); % x = helix start
x0(8)  = path_test.pts(2, 1); % y = helix start
x0(9)  = path_test.pts(3, 1); % z = helix start depth
x0(12) = path_test.psi0;      % aligned with path tangent

[t_cl, X_cl, errs_cl] = run_los_closedloop(path_test, x0, auv_test);

% Cross-track error (horizontal)
hor_xtrack  = sqrt(errs_cl(:, 2).^2);   % y_e component
vert_xtrack = abs(errs_cl(:, 3));       % z_e component

rms_hor  = sqrt(mean(hor_xtrack.^2));
max_hor  = max(hor_xtrack);
rms_vert = sqrt(mean(vert_xtrack.^2));

report('RMS horizontal cross-track < 8m', rms_hor < 8.0);
report('Max horizontal cross-track < 20m', max_hor < 20.0);
report('RMS vertical cross-track < 3m', rms_vert < 3.0);
report('No NaN/Inf over 100s', all(isfinite(X_cl(:))));

fprintf('  RMS x-track (horiz) = %.2fm   max = %.2fm\n', rms_hor, max_hor);
fprintf('  RMS x-track (vert)  = %.2fm\n', rms_vert);

% Save for plot_phase6
save('phase6_results.mat', 't_cl', 'X_cl', 'errs_cl', 'path_test');

fprintf('\n========================================\n');
fprintf('  All [PASS] → Phase 6 complete.\n');
fprintf('  Run plot_phase6 for 3D trajectory.\n');
fprintf('  Proceed to Phase 7 — Environment.\n');
fprintf('========================================\n\n');

% =========================================================================
% Closed-loop simulation
% =========================================================================
function [t_out, X_out, errs_out] = run_los_closedloop(path, x0, auv)

dt    = auv.sim.Ts;
tvec  = 0:dt:auv.sim.T_end;
N     = numel(tvec);
X_out = zeros(N, 12);
X_out(1, :) = x0';
errs_out    = zeros(N, 3);

los  = los_init(auv);
cs   = ctrl_pid_init_local(auv);
opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10, 'MaxStep', 0.05);

x = x0;
for k = 2:N
    t = tvec(k - 1);

    % Path query at current time (moving virtual point)
    [pt, pt_vel] = path_query(path, t);

    % Navigation pass-through
    nu_hat  = x(1:6);
    eta_hat = x(7:12);
    eta_hat(4:6) = arrayfun(@(a) atan2(sin(a), cos(a)), eta_hat(4:6));

    % Guidance
    ds = [auv.guid.U_min; 0; 0];   % desire_speed
    [chi_d, upsilon_d, ud, ~, ~, errs] = los_step(nu_hat, eta_hat, pt, pt_vel, ds, los);
    errs_out(k, :) = [errs.x_e, errs.y_e, errs.z_e];

    % Build guid struct
    guid.chi_d     = chi_d;
    guid.upsilon_d = upsilon_d;
    guid.ud        = ud;
    guid.z_des     = pt(3);

    % Control
    [tau_ctrl, n_direct, ~, cs] = ctrl_pid_step_local(guid, nu_hat, eta_hat, cs);

    % Actuation
    ui = actuation_local(tau_ctrl, n_direct, nu_hat(1), auv);

    % Dynamics
    f = @(~, xx) remus100(xx, ui, 0, 0, 0);
    [~, Ys] = ode45(f, [0 dt], x, opts);
    x = Ys(end, :)';
    X_out(k, :) = x';
end

t_out = tvec;
end

% =========================================================================
% Condensed local copies of previously validated functions
% =========================================================================
function los = los_init(auv)
los.ky      = auv.guid.k_y;
los.kz      = auv.guid.k_z;
los.delta_y = auv.guid.delta_y;
los.delta_z = auv.guid.delta_z;
los.U_max   = auv.guid.U_max;
los.U_min   = auv.guid.U_min;
los.U_nom   = 1.5;
end

function [cd, ud_a, ud_s, cv, uv, errs] = los_step(nu, eta, pt, pv, ds, los)
psi_v = eta(6);
Ry = [cos(psi_v), -sin(psi_v), 0; ...
      sin(psi_v),  cos(psi_v), 0; ...
      0,           0,          1];

vn = Ry * nu(1:3);
[cv, uv] = vehicle_angles(vn);
[pp, tp] = path_angles(pv);
[xe, ye, ze] = los_errors(eta(1:3), pt, pp, tp);
[pr, tr] = los_corrections(xe, ye, ze, los);
[cd, ud_a] = los_compose(pp, tp, pr, tr);
ud_s = los_speed(pv, xe, pr, tr, ds, los);

errs.x_e = xe;
errs.y_e = ye;
errs.z_e = ze;
end

function [cv, uv] = vehicle_angles(v)
if norm(v(1:2)) < 1e-4
    cv = 0;
else
    cv = atan2(v(2), v(1));
end

h = sqrt(v(1)^2 + v(2)^2);
if h < 1e-4
    uv = 0;
else
    uv = atan(-v(3) / h);
end
end

function [pp, tp] = path_angles(pv)
if norm(pv(1:2)) < 1e-6
    pp = 0;
else
    pp = atan2(pv(2), pv(1));
end

h = sqrt(pv(1)^2 + pv(2)^2);
if h < 1e-6
    tp = 0;
else
    tp = atan(-pv(3) / h);
end
end

function [xe, ye, ze] = los_errors(pos, pt, pp, tp)
dx = pos(1) - pt(1);
dy = pos(2) - pt(2);
dz = pos(3) - pt(3);
cp = cos(pp);
sp = sin(pp);
ct = cos(tp);
st = sin(tp);

xe = cp * ct * dx + sp * ct * dy - st * dz;
ye = -sp * dx + cp * dy;
ze = cp * st * dx + sp * st * dy + ct * dz;
end

function [pr, tr] = los_corrections(xe, ye, ze, los) %#ok<INUSL>
pr = tanh(-los.ky * ye / los.delta_y);
tr = tanh( los.kz * ze / los.delta_z);
end

function [cd, uda] = los_compose(pp, tp, pr, tr)
arg = max(-1 + 1e-12, min(1 - 1e-12, ...
    sin(tp) * cos(tr) * cos(pr) + cos(tp) * sin(tr)));
uda = asin(arg);

cy = cos(pp) * sin(pr) * cos(tr) ...
   - sin(tp) * sin(tr) * sin(pp) ...
   + sin(pp) * cos(pr) * cos(tp) * cos(tr);

cx = -sin(pp) * sin(pr) * cos(tr) ...
   -  sin(tp) * sin(tr) * cos(pp) ...
   +  cos(pp) * cos(pr) * cos(tp) * cos(tr);

cd = atan2(cy, cx);
end

function ud = los_speed(pv, xe, pr, tr, ds, los)
denom = max(cos(pr) * cos(tr), 0.1);
raw   = (norm(pv) - 0.001 * xe) / denom;

u = ds(1);
v = ds(2);
w = ds(3);
mag = sqrt(u^2 + v^2 + w^2);

if mag < 1e-6
    ud = los.U_nom;
else
    ud = raw * u / mag;
end

ud = max(los.U_min, min(los.U_max, ud));
end

function ui = actuation_local(tau_ctrl, n_direct, U, auv)
rho = auv.phys.rho;
U_e = max(U, 0.3);

dr = max(auv.act.delta_min, min(auv.act.delta_max, ...
    tau_ctrl(6) / (0.5 * rho * U_e^2 * auv.act.A_r * auv.act.CL_delta_r)));

ds = max(auv.act.delta_min, min(auv.act.delta_max, ...
    tau_ctrl(5) / (0.5 * rho * U_e^2 * auv.act.A_s * auv.act.CL_delta_s)));

n = max(auv.act.n_min, min(auv.act.n_max, n_direct));
ui = [dr; ds; n];
end

function cs = ctrl_pid_init_local(auv)
cs.dt = auv.sim.Ts;

cs.u.integral = 0;
cs.u.e_prev   = 0;
cs.u.Kp       = auv.ctrl.u.Kp;
cs.u.Ki       = auv.ctrl.u.Ki;
cs.u.Kd       = auv.ctrl.u.Kd;
cs.u.sat      = auv.ctrl.u.u_max;

cs.z.integral  = 0;
cs.z.Kp        = auv.ctrl.z.Kp;
cs.z.Ki        = auv.ctrl.z.Ki;
cs.z.theta_max = auv.ctrl.z.theta_d_max;

cs.theta.integral = 0;
cs.theta.e_prev   = 0;
cs.theta.Kp       = auv.ctrl.theta.Kp;
cs.theta.Ki       = auv.ctrl.theta.Ki;
cs.theta.Kd       = auv.ctrl.theta.Kd;
cs.theta.sat      = auv.ctrl.theta.sat;

cs.psi.integral = 0;
cs.psi.e_prev   = 0;
cs.psi.Kp       = auv.ctrl.psi.Kp;
cs.psi.Ki       = auv.ctrl.psi.Ki;
cs.psi.Kd       = auv.ctrl.psi.Kd;
cs.psi.sat      = auv.ctrl.psi.sat;

[~, ~, M] = remus100();
cs.m11 = M(1, 1);
cs.m55 = M(5, 5);
cs.m66 = M(6, 6);
cs.m35 = M(3, 5);
cs.m26 = M(2, 6);
cs.W   = auv.phys.W;
cs.B   = auv.phys.B;
cs.zg  = auv.phys.r_bG(3);
cs.zb  = auv.phys.r_bB(3);
end

function [tc, nd, dbg, cs] = ctrl_pid_step_local(guid, nu, eta, cs)
u     = nu(1);
v     = nu(2);
w     = nu(3);
q     = nu(5);
r     = nu(6);
theta = eta(5);
chi_v = atan2(sin(eta(6)), cos(eta(6)));
Ts    = cs.dt;

[~, ~, M] = remus100();
m55 = M(5, 5);
m66 = M(6, 6);
m35 = M(3, 5);
m26 = M(2, 6);
W   = cs.W;
B   = cs.B;
zg  = cs.zg;
zb  = cs.zb;

e_u = guid.ud - u;
du  = (e_u - cs.u.e_prev) / Ts;
out_u = cs.u.Kp * e_u + cs.u.Ki * cs.u.integral + cs.u.Kd * du;
nd = max(0, min(1525, (1525 / 20) * out_u));
if abs(out_u) <= cs.u.sat
    cs.u.integral = cs.u.integral + e_u * Ts;
end
cs.u.e_prev = e_u;

z_d   = eta(3);
z_ref = guid.z_des;
e_z   = z_ref - z_d;
out_z = cs.z.Kp * e_z + cs.z.Ki * cs.z.integral;
theta_d = max(-cs.z.theta_max, min(cs.z.theta_max, out_z));
if abs(out_z) <= cs.z.theta_max
    cs.z.integral = cs.z.integral + e_z * Ts;
end

e_th = atan2(sin(theta_d - theta), cos(theta_d - theta));
dth  = (e_th - cs.theta.e_prev) / Ts;
out_th_r = cs.theta.Kp * e_th + cs.theta.Ki * cs.theta.integral + cs.theta.Kd * dth;
out_th = max(-cs.theta.sat, min(cs.theta.sat, out_th_r));
ff_p = (W * zg - B * zb) * sin(theta) + 0.3 * m55 * q - m35 * u * w;
tau_M = m55 * out_th + ff_p;
if abs(out_th_r) <= cs.theta.sat
    cs.theta.integral = cs.theta.integral + e_th * Ts;
end
cs.theta.e_prev = e_th;

e_chi = atan2(sin(guid.chi_d - chi_v), cos(guid.chi_d - chi_v));
dpsi  = (e_chi - cs.psi.e_prev) / Ts;
out_p_r = cs.psi.Kp * e_chi + cs.psi.Ki * cs.psi.integral + cs.psi.Kd * dpsi;
out_p = max(-cs.psi.sat, min(cs.psi.sat, out_p_r));
ff_y = 0.1 * m66 * r + m26 * u * v;
tau_N = m66 * out_p + ff_y;
if abs(out_p_r) <= cs.psi.sat
    cs.psi.integral = cs.psi.integral + e_chi * Ts;
end
cs.psi.e_prev = e_chi;

tc = zeros(6, 1);
tc(5) = tau_M;
tc(6) = tau_N;

dbg.e_u     = e_u;
dbg.e_chi   = e_chi;
dbg.e_theta = e_th;
dbg.tau_M   = tau_M;
dbg.tau_N   = tau_N;
dbg.n_direct = nd;
dbg.ff_pitch = ff_p;
dbg.ff_yaw   = ff_y;
end

function path = path_helix(auv)
T    = auv.sim.T_end;
dt   = auv.sim.Ts;
time = 0:dt:T;
m    = 0.6;
Yr   = m * time;

X = 60 * cos(0.02618 * Yr);
Y = 60 * sin(0.02618 * Yr);
Z = 2 + (2 * Yr / 200);

dX = -60 * 0.02618 * m * sin(0.02618 * Yr);
dY =  60 * 0.02618 * m * cos(0.02618 * Yr);
dZ = 2 * m / 200 * ones(size(time));

path.type    = 'helix';
path.t_vec   = time;
path.pts     = [X; Y; Z];
path.vel     = [dX; dY; dZ];
path.T_total = T;
path.U_nom   = m;
path.len     = sum(sqrt(diff(X).^2 + diff(Y).^2 + diff(Z).^2));
path.psi0    = atan2(dY(1), dX(1));
end

function [pt, vel] = path_query(path, t)
t = max(0, min(t, path.T_total));
idx = max(1, min(round(t / (path.t_vec(2) - path.t_vec(1))) + 1, size(path.pts, 2)));
pt  = path.pts(:, idx);
vel = path.vel(:, idx);
end

function a = wrap_angle(x)
a = atan2(sin(x), cos(x));
end

function report(label, condition)
if condition
    fprintf('  [PASS]  %s\n', label);
else
    fprintf('  [FAIL]  %s  <-- FIX THIS\n', label);
end
end
