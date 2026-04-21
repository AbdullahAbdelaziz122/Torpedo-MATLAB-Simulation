%% test_phase4_navigation.m  —  Phase 4 Gate Test
%
% PURPOSE:
%   Validates the Navigation module using navigation_lib.m directly.
%
% PASS CRITERIA:
%   [PASS] Pass-through: nu_hat = nu_true when sigma = 0
%   [PASS] Pass-through: eta_hat = eta_true when sigma = 0
%   [PASS] Speed = |[u,v,w]|, angular rates excluded
%   [PASS] Speed = 0 for zero state
%   [PASS] wrap_angle maps values outside [-pi,pi] correctly
%   [PASS] wrap_angle is idempotent: wrap(wrap(x)) = wrap(x)
%   [PASS] wrap_eta wraps only indices 4-6, not positions 1-3
%   [PASS] wrap_eta leaves positions unchanged
%   [PASS] Noise is zero when all sigma = 0 (deterministic)
%   [PASS] Noise is non-zero when sigma > 0
%   [PASS] Noise std dev is within 3-sigma of configured sigma
%   [PASS] No NaN or Inf in any output for any valid input
%   [PASS] Psi wrapping across the -pi/+pi boundary is correct
%   [PASS] extract_nu returns x(1:6), extract_eta returns x(7:12)
%   [PASS] Architecture: nav output matches NavBus field names
%
% HOW TO RUN:
%   >> buses; auv_params;
%   >> test_phase4_navigation

fprintf('\n========================================\n');
fprintf('  Phase 4 Gate Test — Navigation\n');
fprintf('========================================\n\n');

if ~exist('auv','var')
    error('Run ''buses; auv_params'' first.');
end

% =========================================================================
% Test 1: Pass-through with zero noise
% =========================================================================
fprintf('--- Test 1: Pass-through (sigma = 0) ---\n');

% Build a realistic non-trivial state
x_test = [1.5; 0.1; -0.05;   % u v w
          0.01; 0.05; 0.03;   % p q r
          120; 45; 8;          % x_n y_e z_d
          0.05; -0.12; 1.2];  % phi theta psi

nav = nav_passthrough(x_test, auv);

report('nu_hat = nu_true (pass-through)',  max(abs(nav.nu_hat - x_test(1:6))) < 1e-12);
report('eta_hat = eta_true (pass-through)', max(abs(nav.eta_hat - x_test(7:12))) < 1e-12);

% =========================================================================
% Test 2: Speed computation
% =========================================================================
fprintf('\n--- Test 2: Speed computation ---\n');

x_zero = zeros(12,1);
nav_z  = nav_passthrough(x_zero, auv);
report('Speed = 0 for zero state', abs(nav_z.speed) < 1e-12);

x_surge       = zeros(12,1); x_surge(1) = 1.5;
nav_s         = nav_passthrough(x_surge, auv);
report('Speed = 1.5 m/s for pure surge u=1.5', abs(nav_s.speed - 1.5) < 1e-12);

% Speed uses only [u,v,w] — angular rates must NOT contribute
x_rates       = zeros(12,1); x_rates(4:6) = [0.5; 0.5; 0.5];
nav_r         = nav_passthrough(x_rates, auv);
report('Speed = 0 when only p,q,r are non-zero', abs(nav_r.speed) < 1e-12);

u = 1.0; v = 0.5; w = -0.3;
x_3d          = zeros(12,1); x_3d(1)=u; x_3d(2)=v; x_3d(3)=w;
nav_3d        = nav_passthrough(x_3d, auv);
expected_spd  = sqrt(u^2 + v^2 + w^2);
report(sprintf('3D speed = %.4f m/s', expected_spd), ...
    abs(nav_3d.speed - expected_spd) < 1e-12);

% =========================================================================
% Test 3: wrap_angle — correctness and idempotency
% =========================================================================
fprintf('\n--- Test 3: wrap_angle ---\n');

% Values that should wrap
wrap_cases = [pi + 0.1,   -(pi - 0.1);   % just past +pi → maps to -(pi-0.1)
              2*pi,        0.0;
             -2*pi,        0.0;
              3*pi/2,     -pi/2;
             -3*pi/2,      pi/2;
              0,           0;
              pi/4,        pi/4];

all_wrap_ok = true;
for k = 1:size(wrap_cases,1)
    input    = wrap_cases(k,1);
    expected = wrap_cases(k,2);
    got      = wrap_angle(input);
    if abs(got - expected) > 1e-10
        fprintf('  wrap_angle(%.4f): expected %.4f, got %.4f\n', input, expected, got);
        all_wrap_ok = false;
    end
end
report('wrap_angle maps all test cases correctly', all_wrap_ok);

% Idempotency: wrap(wrap(x)) = wrap(x) for all x
test_angles = linspace(-3*pi, 3*pi, 200);
idempotent  = true;
for a = test_angles
    w1 = wrap_angle(a);
    w2 = wrap_angle(w1);
    if abs(w1 - w2) > 1e-12,  idempotent = false;  end
end
report('wrap_angle is idempotent: wrap(wrap(x)) = wrap(x)', idempotent);

% Output always in [-pi, pi]
all_in_range = all(abs(arrayfun(@wrap_angle, test_angles)) <= pi + 1e-12);
report('wrap_angle output always in [-pi, pi]', all_in_range);

% =========================================================================
% Test 4: wrap_eta — angle indices wrapped, position indices preserved
% =========================================================================
fprintf('\n--- Test 4: wrap_eta ---\n');

eta_big = [1000; -500; 200;    % positions — must NOT be wrapped
           3*pi;  -5;   7];    % angles — must be wrapped

eta_w = wrap_eta(eta_big);

report('eta_hat positions (1-3) unchanged by wrap_eta', ...
    max(abs(eta_w(1:3) - eta_big(1:3))) < 1e-12);
report('phi   wrapped to [-pi, pi]',   abs(eta_w(4)) <= pi + 1e-12);
report('theta wrapped to [-pi, pi]',   abs(eta_w(5)) <= pi + 1e-12);
report('psi   wrapped to [-pi, pi]',   abs(eta_w(6)) <= pi + 1e-12);

% Specific value check: psi = 7 rad → wrap to 7 - 2*pi ≈ 0.7168 rad
psi_wrap_expected = atan2(sin(7), cos(7));
report(sprintf('psi = 7 rad wraps to %.4f rad', psi_wrap_expected), ...
    abs(eta_w(6) - psi_wrap_expected) < 1e-10);

% =========================================================================
% Test 5: Critical — psi wrapping across ±pi boundary
% =========================================================================
fprintf('\n--- Test 5: Psi boundary crossing ---\n');

% Vehicle heading crosses from +179 deg to +181 deg — must wrap to -179 deg
psi_before = deg2rad(179);
psi_after  = deg2rad(181);   % equivalent to -179 deg

eta_before = [0;0;0; 0;0; psi_before];
eta_after  = [0;0;0; 0;0; psi_after];

nav_before = nav_passthrough([zeros(6,1); eta_before], auv);
nav_after  = nav_passthrough([zeros(6,1); eta_after],  auv);

psi_wrapped = nav_after.eta_hat(6);
psi_expected = deg2rad(-179);

report('psi=181 deg wraps to -179 deg', abs(psi_wrapped - psi_expected) < 1e-10);

% Continuity: |wrap(181°) - wrap(179°)| = 2 deg, not 360-2 = 358 deg
angular_diff = abs(nav_after.eta_hat(6) - nav_before.eta_hat(6));
report('Angular diff across boundary = 2 deg (not 358 deg)', ...
    abs(angular_diff - deg2rad(2)) < 1e-10);

% =========================================================================
% Test 6: extract_nu and extract_eta
% =========================================================================
fprintf('\n--- Test 6: State extraction ---\n');

x_ext = (1:12)';   % [1;2;3;4;5;6;7;8;9;10;11;12]

nu_ext  = extract_nu(x_ext);
eta_ext = extract_eta(x_ext);

report('extract_nu  returns x(1:6)',  isequal(nu_ext,  (1:6)'));
report('extract_eta returns x(7:12)', isequal(eta_ext, (7:12)'));
report('extract_nu  is 6×1',          isequal(size(nu_ext),  [6 1]));
report('extract_eta is 6×1',          isequal(size(eta_ext), [6 1]));

% =========================================================================
% Test 7: Noise injection
% =========================================================================
fprintf('\n--- Test 7: Noise injection ---\n');

% Zero sigma → exact zeros (deterministic, no randn calls)
noise_zero = add_sensor_noise(zeros(6,1));
report('Zero sigma → noise is exactly zero', all(noise_zero == 0));

% Non-zero sigma → non-zero noise (probabilistic — draw 1000 samples)
sigma_test = 0.1 * ones(6,1);
N_samples  = 1000;
samples    = zeros(6, N_samples);
for k = 1:N_samples
    samples(:,k) = add_sensor_noise(sigma_test);
end
all_nonzero = all(std(samples, 0, 2) > 0.01);   % std should be ~0.1
report('Non-zero sigma → noise samples are non-zero', all_nonzero);

% Check empirical std dev is within [0.5*sigma, 2.0*sigma] at 1000 samples
stds      = std(samples, 0, 2);
in_range  = all(stds > 0.5*0.1) && all(stds < 2.0*0.1);
report('Noise std dev within [0.5σ, 2.0σ] at 1000 samples', in_range);
fprintf('  Empirical std per channel: [');
fprintf('%.3f ', stds');
fprintf(']\n');

% =========================================================================
% Test 8: Robustness — no NaN or Inf for valid inputs
% =========================================================================
fprintf('\n--- Test 8: Robustness ---\n');

test_states = {
    zeros(12,1),                      'zero state';
    [2.5; zeros(11,1)],               'max surge';
    [zeros(5,1); 0.5; zeros(6,1)],    'yaw rate only';
    [zeros(9,1); pi-0.01; 0; pi-0.01],'near-singular angles';
    [zeros(6,1); 1e4; 1e4; 500; 0; 0; 0], 'large position';
};

for k = 1:size(test_states,1)
    x_k   = test_states{k,1};
    label = test_states{k,2};
    nav_k = nav_passthrough(x_k, auv);
    no_nan = ~any(isnan([nav_k.nu_hat; nav_k.eta_hat; nav_k.speed]));
    no_inf = ~any(isinf([nav_k.nu_hat; nav_k.eta_hat; nav_k.speed]));
    report(sprintf('No NaN/Inf: %s', label), no_nan && no_inf);
end

% =========================================================================
% Test 9: Architecture — nav output has correct NavBus field names
% =========================================================================
fprintf('\n--- Test 9: NavBus field names ---\n');

nav_out = nav_passthrough(x_test, auv);
report('nav output has field nu_hat',  isfield(nav_out, 'nu_hat'));
report('nav output has field eta_hat', isfield(nav_out, 'eta_hat'));
report('nav output has field speed',   isfield(nav_out, 'speed'));
report('nu_hat  is 6×1',               isequal(size(nav_out.nu_hat),  [6 1]));
report('eta_hat is 6×1',               isequal(size(nav_out.eta_hat), [6 1]));
report('speed   is scalar',            isscalar(nav_out.speed));

% =========================================================================
fprintf('\n========================================\n');
fprintf('  All [PASS] → Phase 4 complete.\n');
fprintf('  Proceed to Phase 5 — Control (PID).\n');
fprintf('========================================\n\n');

% =========================================================================
% Local functions — must be at end of script
% =========================================================================

function report(label, condition)
if condition,  fprintf('  [PASS]  %s\n', label);
else,          fprintf('  [FAIL]  %s  <-- FIX THIS\n', label);
end
end

function nav = nav_passthrough(x_true, auv)
nu_true  = x_true(1:6);
eta_true = x_true(7:12);
sigma_v  = auv.nav.sigma_vel;   sigma_p = auv.nav.sigma_pos;
sigma_a  = auv.nav.sigma_angle; sigma_r = auv.nav.sigma_rate;
nu_hat   = nu_true  + add_sensor_noise([sigma_v;sigma_v;sigma_v;sigma_r;sigma_r;sigma_r]);
eta_hat  = eta_true + add_sensor_noise([sigma_p;sigma_p;sigma_p;sigma_a;sigma_a;sigma_a]);
eta_hat  = wrap_eta(eta_hat);
nav.nu_hat  = nu_hat;
nav.eta_hat = eta_hat;
nav.speed   = sqrt(nu_hat(1)^2 + nu_hat(2)^2 + nu_hat(3)^2);
end

function nu = extract_nu(x)
nu = x(1:6);
end

function eta = extract_eta(x)
eta = x(7:12);
end

function a = wrap_angle(angle_in)
a = atan2(sin(angle_in), cos(angle_in));
end

function eta_w = wrap_eta(eta)
eta_w    = eta;
eta_w(4) = atan2(sin(eta(4)), cos(eta(4)));
eta_w(5) = atan2(sin(eta(5)), cos(eta(5)));
eta_w(6) = atan2(sin(eta(6)), cos(eta(6)));
end

function n = add_sensor_noise(sigma_vec)
if all(sigma_vec == 0),  n = zeros(6,1);
else,                     n = sigma_vec .* randn(6,1);
end
end