%% test_phase1.m  —  Phase 1 Gate Test
%
% PURPOSE:
%   Validates that all bus objects and parameter structs are correctly
%   defined and internally consistent before any Simulink work begins.
%
% PASS CRITERIA:
%   [PASS] All 9 bus objects exist in base workspace
%   [PASS] StateBus sub-buses match NuBus and EtaBus
%   [PASS] auv.ic.x0 is exactly 12×1 double
%   [PASS] auv.act.delta_max = 0.3491 rad (±20 deg)
%   [PASS] GuidanceBus has exactly 3 elements
%   [PASS] ControlBus tau_ctrl is 6-dimensional
%   [PASS] ActuationBus has sat_flags of uint8
%   [PASS] State vector assembly round-trips cleanly
%   [PASS] PWM mapping is monotonic and within [1000, 2000]
%
% HOW TO RUN:
%   >> buses; auv_params; test_phase1
%
% EXPECTED OUTPUT:
%   All lines print [PASS]. Zero [FAIL] lines.

fprintf('\n========================================\n');
fprintf('  Phase 1 Gate Test\n');
fprintf('========================================\n\n');

pass = 0; fail = 0;


% -------------------------------------------------------------------------
% 1. Bus objects exist in base workspace
% -------------------------------------------------------------------------
bus_names = {'NuBus','EtaBus','StateBus','GuidanceBus', ...
             'CtrlDebugBus','ControlBus','ActuationBus','EnvBus','NavBus'};

fprintf('--- Bus object existence ---\n');
for k = 1:numel(bus_names)
    name = bus_names{k};
    exists = evalin('base', sprintf('exist(''%s'',''var'')', name));
    report(sprintf('%s exists', name), exists == 1);
end

% -------------------------------------------------------------------------
% 2. Bus element counts
% -------------------------------------------------------------------------
fprintf('\n--- Bus element counts ---\n');

NuBus_     = evalin('base', 'NuBus');
EtaBus_    = evalin('base', 'EtaBus');
StateBus_  = evalin('base', 'StateBus');
GuidBus_   = evalin('base', 'GuidanceBus');
CtrlBus_   = evalin('base', 'ControlBus');
ActBus_    = evalin('base', 'ActuationBus');
EnvBus_    = evalin('base', 'EnvBus');
NavBus_    = evalin('base', 'NavBus');

% Verify that StateBus has 2 elements
nu_elem = StateBus_.Elements(1);
eta_elem = StateBus_.Elements(2);
report('StateBus.nu is Bus: NuBus', strcmp(nu_elem.DataType, 'Bus: NuBus'));
report('StateBus.eta is Bus: EtaBus', strcmp(eta_elem.DataType, 'Bus: EtaBus'));

report('NuBus  has 6 elements',        numel(NuBus_.Elements)    == 6);
report('EtaBus has 6 elements',        numel(EtaBus_.Elements)   == 6);
report('StateBus has 2 sub-buses',     numel(StateBus_.Elements) == 2);
report('GuidanceBus has 3 elements',   numel(GuidBus_.Elements)  == 3);
report('ControlBus has 2 elements',    numel(CtrlBus_.Elements)  == 2);
report('ActuationBus has 7 elements',  numel(ActBus_.Elements)   == 7);
report('EnvBus has 4 elements',        numel(EnvBus_.Elements)   == 4);
report('NavBus has 3 elements',        numel(NavBus_.Elements)   == 3);

% -------------------------------------------------------------------------
% MSS Toolbox prerequisite
% -------------------------------------------------------------------------

fprintf('\n--- MSS Toolbox prerequisite ---\n');
try
    [~,~,M] = remus100();
    report('remus100() returns a mass matrix', size(M,1)==6 && size(M,2)==6);
catch
    report('remus100() is callable (MSS Toolbox installed)', false);
    fprintf('  ERROR: MSS Toolbox not found or not on path.\n');
    fprintf('  Run mssstart from the MSS root folder.\n');
end

% -------------------------------------------------------------------------
% 3. GuidanceBus element names are exactly correct
% -------------------------------------------------------------------------
fprintf('\n--- GuidanceBus element names ---\n');
g_names = {GuidBus_.Elements.Name};
report('GuidanceBus has chi_d',      any(strcmp(g_names,'chi_d')));
report('GuidanceBus has upsilon_d',  any(strcmp(g_names,'upsilon_d')));
report('GuidanceBus has ud',         any(strcmp(g_names,'ud')));

% -------------------------------------------------------------------------
% 4. ControlBus tau_ctrl is 6-dimensional
% -------------------------------------------------------------------------
fprintf('\n--- ControlBus validation ---\n');
tau_elem = CtrlBus_.Elements(strcmp({CtrlBus_.Elements.Name}, 'tau_ctrl'));
report('ControlBus.tau_ctrl dimension = 6', tau_elem.Dimensions == 6);
report('ControlBus.tau_ctrl type is double', ...
    strcmp(tau_elem.DataType,'double'));

% -------------------------------------------------------------------------
% 5. ActuationBus sat_flags is uint8
% -------------------------------------------------------------------------
fprintf('\n--- ActuationBus validation ---\n');
sf_elem = ActBus_.Elements(strcmp({ActBus_.Elements.Name}, 'sat_flags'));
report('ActuationBus.sat_flags is uint8', strcmp(sf_elem.DataType,'uint8'));
report('ActuationBus.sat_flags dimension = 3', sf_elem.Dimensions == 3);

% -------------------------------------------------------------------------
% 6. auv parameter struct validation
% -------------------------------------------------------------------------
fprintf('\n--- auv parameter struct ---\n');
auv_ = evalin('base', 'auv');

report('auv.ic.x0 is 12×1', ...
    isequal(size(auv_.ic.x0), [12 1]));
report('auv.ic.x0 is all zeros (default IC)', ...
    all(auv_.ic.x0 == 0));
report('auv.act.delta_max ≈ 0.3491 rad (20 deg)', ...
    abs(auv_.act.delta_max - deg2rad(20)) < 1e-10);
report('auv.act.n_max = 1525 RPM', ...
    auv_.act.n_max == 1525);
report('auv.phys.m = 31.9 kg', ...
    abs(auv_.phys.m - 31.9) < 1e-9);
report('auv.sim.Ts = 0.01 s', ...
    abs(auv_.sim.Ts - 0.01) < 1e-12);
report('auv.phys.L = 1.6 m', ...
    abs(auv_.phys.L - 1.6) < 1e-9);

% -------------------------------------------------------------------------
% 7. State vector convention and assembly
% -------------------------------------------------------------------------
fprintf('\n--- State vector convention ---\n');
nu_test  = [1.5; 0.1; -0.05; 0.01; 0.02; 0.03];
eta_test = [10; 5; 3; 0.01; 0.05; 1.2];
x_test   = [nu_test; eta_test];   % remus100.m ordering: nu first

report('State vector is 12×1', isequal(size(x_test), [12 1]));
report('nu = x(1:6), eta = x(7:12)', ...
    isequal(x_test(1:6), nu_test) && isequal(x_test(7:12), eta_test));
report('u = x(1) matches', x_test(1) == nu_test(1));
report('psi = x(12) matches', x_test(12) == eta_test(6));

% -------------------------------------------------------------------------
% 8. PWM mapping monotonicity
% -------------------------------------------------------------------------
fprintf('\n--- PWM mapping ---\n');
% Rudder: delta ∈ [-delta_max, +delta_max] → PWM ∈ [1000, 2000]
assert(auv_.act.delta_max > 0, 'delta_max must be positive');
d_max  = auv_.act.delta_max;
pwm_n  = auv_.act.pwm_fin_neutral;
pwm_r  = auv_.act.pwm_fin_range;

pwm_at_neg = pwm_n + (-d_max / d_max) * pwm_r;  % 1000
pwm_at_pos = pwm_n + ( d_max / d_max) * pwm_r;  % 2000

report('PWM at -delta_max = 1000 µs', abs(pwm_at_neg - 1000) < 1e-9);
report('PWM at +delta_max = 2000 µs', abs(pwm_at_pos - 2000) < 1e-9);
report('ESC PWM range valid [1000,2000]', ...
    auv_.act.pwm_esc_min == 1000 && auv_.act.pwm_esc_max == 2000);

% -------------------------------------------------------------------------
% 9. Physical consistency checks
% -------------------------------------------------------------------------
fprintf('\n--- Physical consistency ---\n');
W = auv_.phys.W;
B = auv_.phys.B;
report('W = m*g computed correctly', ...
    abs(W - auv_.phys.m * auv_.phys.g) < 1e-6);
report('Neutral buoyancy: |W-B|/W < 1%', abs(W-B)/W < 0.01);
report('Iz = Iy (axisymmetric body)', ...
    abs(auv_.phys.Iz - auv_.phys.Iy) < 1e-9);
report('|Xudot| << |Yvdot| (slender body)', ...
    abs(auv_.added.Xudot) < abs(auv_.added.Yvdot));
report('Zwdot = Yvdot (axisymmetric added mass)', ...
    abs(auv_.added.Zwdot - auv_.added.Yvdot) < 1e-9);

% -------------------------------------------------------------------------
% Summary
% -------------------------------------------------------------------------
fprintf('\n========================================\n');
fprintf('  If all lines show [PASS]: proceed to Phase 2\n');
fprintf('  If any [FAIL] exists: fix before continuing\n');
fprintf('========================================\n\n');
fprintf('Next command:\n');
fprintf('  Phase 2 -> Build Dynamics_M5 Simulink subsystem\n');
fprintf('  Requires: MSS Toolbox on MATLAB path\n');
fprintf('  Verify:   >> which remus100    (must return a path)\n\n');

% =========================================================================
% LOCAL FUNCTIONS (Must be at the end of the script)
% =========================================================================
function report(label, condition)
    if condition
        fprintf('  [PASS]  %s\n', label);
    else
        fprintf('  [FAIL]  %s  <-- FIX THIS\n', label);
    end
end
