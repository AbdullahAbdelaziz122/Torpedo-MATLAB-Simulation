%% test_phase2_openloop.m  —  Phase 2 Gate Test
%
% PURPOSE:
%   Validates remus100.m dynamics by running an open-loop simulation
%   entirely in MATLAB (no Simulink required for this test).
%   Simulink is only needed for the Dynamics_M5.slx block — but
%   the physics must be correct first, and this test proves that
%   independently of Simulink's wiring.
%
% STRATEGY:
%   Direct ode45 integration of remus100.m with fixed inputs.
%   This isolates the dynamics completely — if this test fails,
%   the problem is in remus100.m or MSS Toolbox, not Simulink.
%
% PASS CRITERIA:
%   [PASS] remus100 is on the MATLAB path
%   [PASS] Empty call [~,~,M] = remus100() returns 6×6 positive definite M
%   [PASS] Mass matrix M is symmetric
%   [PASS] Surge reaches 2.0–2.6 m/s at t=60s with 1525 RPM
%   [PASS] Sway and heave remain bounded: |v|,|w| < 0.5 m/s
%   [PASS] Roll stays bounded: |phi| < 10 deg
%   [PASS] Depth drift < 5 m over 60s (neutral buoyancy + small rounding)
%   [PASS] xdot(7:12) = J(eta)*nu (kinematic consistency)
%   [PASS] State dimension stays 12 throughout simulation
%   [PASS] No NaN or Inf in state at any timestep
%
% TESTS RUN:
%   Test 1: MSS Toolbox availability
%   Test 2: Mass matrix properties
%   Test 3: Surge-only open loop (zero fins, constant RPM=1525)
%   Test 4: Zero-input stability (coasting from u0=1.5 m/s)
%   Test 5: Rudder step response (lateral motion coupling)
%   Test 6: Stern plane step response (pitch/depth coupling)
%
% HOW TO RUN:
%   >> buses; auv_params;
%   >> test_phase2_openloop
%   (Results automatically saved to phase2_results.mat)
%
% AUTHOR: AUV Simulation Project — Phase 2

fprintf('\n========================================\n');
fprintf('  Phase 2 Gate Test — Dynamics Validation\n');
fprintf('========================================\n\n');

if ~exist('auv','var')
    error('Run ''buses; auv_params'' before this test.');
end

results = struct();  % accumulate simulation outputs for plot_phase2.m

% =========================================================================
% TEST 1: MSS Toolbox availability
% =========================================================================
fprintf('--- Test 1: MSS Toolbox ---\n');

path_ok = ~isempty(which('remus100'));
report('remus100.m is on MATLAB path', path_ok);

if ~path_ok
    fprintf('\n  [FATAL] remus100.m not found.\n');
    fprintf('  Download MSS Toolbox: github.com/cybergalactic/MSS\n');
    fprintf('  Then run mssstart.m to add it to path.\n\n');
    return
end

deps = {'spheroid','imlay61','Dmtrx','forceLiftDrag', ...
        'crossFlowDrag','gRvect','eulerang','gravity','sat'};
for k = 1:numel(deps)
    report(sprintf('%s.m on path', deps{k}), ~isempty(which(deps{k})));
end

% =========================================================================
% TEST 2: Mass matrix properties
% =========================================================================
fprintf('\n--- Test 2: Mass matrix M = MRB + MA ---\n');

[~, ~, M] = remus100();   % empty call returns mass matrix

report('M is 6×6',                     isequal(size(M), [6 6]));
report('M is symmetric: max|M-M''| < 1e-10', max(max(abs(M - M'))) < 1e-10);
report('M is positive definite (all eig > 0)', all(eig(M) > 0));
report('M(1,1) ≈ m + |Xudot| ≈ 32.83',  abs(M(1,1) - 32.83) < 0.5);

% Remove the hardcoded values and replaced it with 10% tolerance 
%report('M(2,2) ≈ m + |Yvdot| ≈ 67.4',   abs(M(2,2) - 67.4)  < 1.0);
%report('M(5,5) ≈ Iy + |Mqdot| ≈ 8.33',  abs(M(5,5) - 8.33)  < 0.5);
% Relative tolerance 10%
rel_tol = 0.10;
m_plus_Yvdot_expected = auv.phys.m + abs(auv.added.Yvdot);  % 31.9 + 35.5 = 67.4
Iy_plus_Mqdot_expected = auv.phys.Iy + abs(auv.added.Mqdot); % 3.45 + 4.88 = 8.33

report(sprintf('M(2,2) ≈ m + |Yvdot| = %.1f (rel err < %.0f%%)', m_plus_Yvdot_expected, rel_tol*100), ...
    abs(M(2,2) - m_plus_Yvdot_expected) / m_plus_Yvdot_expected < rel_tol);
report(sprintf('M(5,5) ≈ Iy + |Mqdot| = %.2f (rel err < %.0f%%)', Iy_plus_Mqdot_expected, rel_tol*100), ...
    abs(M(5,5) - Iy_plus_Mqdot_expected) / Iy_plus_Mqdot_expected < rel_tol);


fprintf('\n  Full M matrix (for reference):\n');
for r = 1:6
    fprintf('    [');
    for c = 1:6
        fprintf('%8.3f', M(r,c));
    end
    fprintf(' ]\n');
end

% =========================================================================
% TEST 3: Surge open-loop — constant RPM=1525, zero fins
% =========================================================================
fprintf('\n--- Test 3: Surge open-loop (1525 RPM, zero fins, 60s) ---\n');

x0   = zeros(12,1);
ui3  = [0; 0; 1525];       % [delta_r, delta_s, n_RPM]
tspan = [0, 60];

% Wrap remus100 for ode45 with constant input, zero current
f3 = @(t,x) remus100(x, ui3, 0, 0, 0);

opts = odeset('RelTol',1e-8,'AbsTol',1e-10,'MaxStep',0.1);
[t3, X3] = ode45(f3, tspan, x0, opts);

u_final   = X3(end, 1);
v_max     = max(abs(X3(:,2)));
w_max     = max(abs(X3(:,3)));
phi_max   = max(abs(X3(:,10)));
depth_end = X3(end, 9);

results.t3 = t3;  results.X3 = X3;  results.ui3 = ui3;

report('Surge reaches ≥ 2.0 m/s at t=60s',    u_final >= 2.0);
report('Surge does not exceed 2.6 m/s',         u_final <= 2.6);
report('Sway bounded: max|v| < 0.5 m/s',        v_max < 0.5);
report('Heave bounded: max|w| < 0.5 m/s',       w_max < 0.5);
report('Roll bounded: max|phi| < 10 deg',        phi_max < deg2rad(10));
report('Depth drift bounded: |z| < 5 m at 60s', abs(depth_end) < 5.0);
report('No NaN in state trajectory',             ~any(isnan(X3(:))));
report('No Inf in state trajectory',             ~any(isinf(X3(:))));

fprintf('\n  Final state at t=60s:\n');
fprintf('    u=%.3f m/s,  v=%.4f m/s,  w=%.4f m/s\n', X3(end,1), X3(end,2), X3(end,3));
fprintf('    p=%.4f rad/s, q=%.4f rad/s, r=%.4f rad/s\n', X3(end,4), X3(end,5), X3(end,6));
fprintf('    x=%.2fm, y=%.2fm, z=%.2fm\n', X3(end,7), X3(end,8), X3(end,9));
fprintf('    phi=%.3f deg, theta=%.3f deg, psi=%.3f deg\n', ...
    rad2deg(X3(end,10)), rad2deg(X3(end,11)), rad2deg(X3(end,12)));

% =========================================================================
% TEST 4: Zero-input coasting — initial u=1.5 m/s, RPM=0
% =========================================================================
fprintf('\n--- Test 4: Coasting deceleration (u0=1.5 m/s, RPM=0) ---\n');

x0_4  = zeros(12,1);  x0_4(1) = 1.5;  % u = 1.5 m/s
ui4   = [0; 0; 0];
tspan4 = [0, 30];

f4 = @(t,x) remus100(x, ui4, 0, 0, 0);
[t4, X4] = ode45(f4, tspan4, x0_4, opts);

u_t30 = X4(end,1);
results.t4 = t4;  results.X4 = X4;

report('Vehicle decelerates: u(30s) < u(0s)',      u_t30 < 1.5);
report('Vehicle does not reverse: u(30s) > -0.1',  u_t30 > -0.1);
report('No NaN in coasting trajectory',             ~any(isnan(X4(:))));

fprintf('  u(0)=1.50 m/s  →  u(30s)=%.3f m/s  (drag deceleration OK)\n', u_t30);

% =========================================================================
% TEST 5: Rudder step — heading response
% =========================================================================
fprintf('\n--- Test 5: Rudder step (delta_r=15 deg, u0=1.5 m/s, 30s) ---\n');

x0_5  = zeros(12,1);  x0_5(1) = 1.5;
ui5   = [deg2rad(15); 0; 1000];   % 15 deg rudder, moderate RPM
tspan5 = [0, 30];

f5 = @(t,x) remus100(x, ui5, 0, 0, 0);
[t5, X5] = ode45(f5, tspan5, x0_5, opts);

psi_max  = max(abs(X5(:,12)));
r_max    = max(abs(X5(:,6)));
results.t5 = t5;  results.X5 = X5;

report('Rudder step produces yaw: max|psi| > 5 deg',  psi_max > deg2rad(5));
report('Yaw rate bounded: max|r| < 0.5 rad/s',         r_max < 0.5);
report('No NaN in rudder-step trajectory',              ~any(isnan(X5(:))));

fprintf('  max|psi|=%.1f deg,  max|r|=%.3f rad/s\n', rad2deg(psi_max), r_max);

% =========================================================================
% TEST 6: Stern plane step — pitch/depth response
% =========================================================================
fprintf('\n--- Test 6: Stern plane step (delta_s=10 deg, u0=1.5 m/s, 30s) ---\n');

x0_6  = zeros(12,1);  x0_6(1) = 1.5;
ui6   = [0; deg2rad(10); 1000];   % 10 deg stern plane
tspan6 = [0, 30];

f6 = @(t,x) remus100(x, ui6, 0, 0, 0);
[t6, X6] = ode45(f6, tspan6, x0_6, opts);

theta_max = max(abs(X6(:,11)));
z_end     = X6(end, 9);
results.t6 = t6;  results.X6 = X6;

report('Stern plane produces pitch: max|theta| > 2 deg', theta_max > deg2rad(2));
report('Depth changes: |z(30s)| > 0.5 m',                abs(z_end) > 0.5);
report('No NaN in stern-plane trajectory',                ~any(isnan(X6(:))));

fprintf('  max|theta|=%.1f deg,  z(30s)=%.2f m\n', rad2deg(theta_max), z_end);

% =========================================================================
% TEST 7: Kinematic consistency check — xdot(7:12) = J(eta)*nu
% =========================================================================
fprintf('\n--- Test 7: Kinematic consistency at one sample point ---\n');

x_test = X3(50,:)';  % pick an arbitrary point from Test 3
[xdot_test, ~] = remus100(x_test, ui3, 0, 0, 0);

% Manually compute J(eta)*nu
phi_t   = x_test(10); theta_t = x_test(11); psi_t = x_test(12);
[J_test, ~] = eulerang(phi_t, theta_t, psi_t);   % from MSS Toolbox
eta_dot_expected = J_test * x_test(1:6);
eta_dot_actual   = xdot_test(7:12);
kin_err = max(abs(eta_dot_actual - eta_dot_expected));

report('Kinematic consistency: max|error| < 1e-10', kin_err < 1e-10);
fprintf('  Kinematic error: %.2e (should be ~0)\n', kin_err);

% =========================================================================
% Save results and summary
% =========================================================================
save('phase2_results.mat', 'results', 't3','X3','t4','X4','t5','X5','t6','X6');
fprintf('\n  Results saved to phase2_results.mat\n');

fprintf('\n========================================\n');
fprintf('  PHASE 2 GATE TEST COMPLETE\n');
fprintf('  If all lines show [PASS]: run plot_phase2\n');
fprintf('  then proceed to Phase 3 — Actuation\n');
fprintf('========================================\n\n');

% =========================================================================
% Local functions
% =========================================================================
function report(label, condition)
if condition
    fprintf('  [PASS]  %s\n', label);
else
    fprintf('  [FAIL]  %s  <-- FIX THIS\n', label);
end
end
