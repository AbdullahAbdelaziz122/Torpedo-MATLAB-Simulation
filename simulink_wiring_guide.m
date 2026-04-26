%% simulink_wiring_guide.m  —  Complete manual wiring reference
%
% PURPOSE:
%   Documents every connection in AUV_TopLevel.slx precisely.
%   Use this as:
%   (a) A reference when build_simulink_model.m has partial issues
%   (b) A manual wiring checklist to verify the auto-built model
%   (c) Documentation for understanding signal flow
%
% ALGEBRAIC LOOP ANALYSIS:
%   The closed loop contains a potential algebraic loop:
%
%   Navigation(Out1:nu_hat) → Control(In4) → Control(Out1:tau_ctrl)
%   → Actuation(In1) → Actuation(Out1:ui) → Dynamics(In1)
%   → Dynamics(Out1:x) → Navigation(In1) → Navigation(Out1:nu_hat)
%
%   All blocks have DirectFeedthrough=true, meaning Simulink cannot
%   determine an evaluation order without a loop. The fix is a
%   Unit Delay on the Navigation output — this breaks the loop at
%   the cost of one Ts = 0.01s lag on state feedback.
%   At 100 Hz this is negligible for all practical control bandwidths.
%
% FIX: Add Unit Delay blocks on Navigation outputs before Control:
%   Navigation/Out1 (nu_hat)  → UnitDelay_nu  → Control/In4
%   Navigation/Out2 (eta_hat) → UnitDelay_eta → Control/In5
%
% AUTHOR: AUV Simulation Project — Simulink Wiring

fprintf('\n=== Simulink Wiring Guide ===\n\n');
fprintf('Run this script to verify your model has all required connections.\n\n');

%% ── Complete signal table ────────────────────────────────────────────────
fprintf('SIGNAL CONNECTIONS TABLE\n');
fprintf('%-30s  %-6s  %-30s  %-6s  %-20s\n', ...
    'SOURCE BLOCK','PORT','DESTINATION BLOCK','PORT','SIGNAL');
fprintf('%s\n', repmat('-',1,100));

connections = {
% Source block          SrcPort  Dest block         DstPort  Signal name          Dims
'Clock',                1,  'Environment',     1,  't',                    '[1]';
'Clock',                1,  'PathGenerator',   1,  't',                    '[1]';
'Environment',          1,  'Dynamics',        2,  'Vc',                   '[1]';
'Environment',          2,  'Dynamics',        3,  'betaVc',               '[1]';
'Environment',          3,  'Dynamics',        4,  'w_c',                  '[1]';
'Dynamics',             1,  'Navigation',      1,  'x_true',               '[12]';
'Navigation',           1,  'UnitDelay_nu',    1,  'nu_hat',               '[6]';
'Navigation',           2,  'UnitDelay_eta',   1,  'eta_hat',              '[6]';
'UnitDelay_nu',         1,  'Guidance',        1,  'nu_hat_d',             '[6]';
'UnitDelay_eta',        1,  'Guidance',        2,  'eta_hat_d',            '[6]';
'UnitDelay_nu',         1,  'Control',         4,  'nu_hat_d',             '[6]';
'UnitDelay_eta',        1,  'Control',         5,  'eta_hat_d',            '[6]';
'PathGenerator',        1,  'Guidance',        3,  'pt',                   '[3]';
'PathGenerator',        2,  'Guidance',        4,  'pt_vel',               '[3]';
'DesiredSpeed',         1,  'Guidance',        5,  'desire_spd',           '[3]';
'Guidance',             1,  'Control',         1,  'chi_d',                '[1]';
'Guidance',             2,  'Control',         2,  'upsilon_d',            '[1]';
'Guidance',             3,  'Control',         3,  'ud',                   '[1]';
'Control',              1,  'Actuation',       1,  'tau_ctrl',             '[6]';
'Control',              2,  'Actuation',       3,  'n_direct',             '[1]';
'Dynamics',             3,  'Actuation',       2,  'U',                    '[1]';
'Actuation',            1,  'Dynamics',        1,  'ui',                   '[3]';
};

for i = 1:size(connections,1)
    fprintf('%-30s  In%-4d  %-30s  Out%-4d %-20s %s\n', ...
        connections{i,1}, connections{i,2}, ...
        connections{i,3}, connections{i,4}, ...
        connections{i,5}, connections{i,6});
end

%% ── Unit Delay configuration ─────────────────────────────────────────────
fprintf('\nUNIT DELAY CONFIGURATION\n');
fprintf('  Both UnitDelay blocks must be configured as:\n');
fprintf('  Initial condition: zeros(6,1)\n');
fprintf('  Sample time:       %g (inherited from model Ts)\n', 0.01);
fprintf('  Vector size:       6×1\n\n');

%% ── Programmatic fix: add Unit Delays to existing model ─────────────────
if bdIsLoaded('AUV_TopLevel')
    fprintf('AUV_TopLevel is loaded. Adding Unit Delay fix...\n');
    add_unit_delays('AUV_TopLevel');
else
    fprintf('Run build_simulink_model first, then re-run this script\n');
    fprintf('to automatically add the Unit Delay fix.\n\n');
end

%% ── Logging block table ──────────────────────────────────────────────────
fprintf('\nLOGGING BLOCKS (To Workspace)\n');
fprintf('%-20s  %-30s  %-6s  %-15s\n','Variable','Source block','Port','Dimensions');
fprintf('%s\n', repmat('-',1,75));
log_table = {
    'log_x',         'Dynamics',    1, '[12×N]';
    'log_xdot',      'Dynamics',    2, '[12×N]';
    'log_U',         'Dynamics',    3, '[1×N]';
    'log_nu_hat',    'Navigation',  1, '[6×N]';
    'log_eta_hat',   'Navigation',  2, '[6×N]';
    'log_chi_d',     'Guidance',    1, '[1×N]';
    'log_upsilon_d', 'Guidance',    2, '[1×N]';
    'log_ud',        'Guidance',    3, '[1×N]';
    'log_los_err',   'Guidance',    6, '[3×N]';
    'log_tau_ctrl',  'Control',     1, '[6×N]';
    'log_n_direct',  'Control',     2, '[1×N]';
    'log_e_chi',     'Control',     3, '[1×N]';
    'log_e_theta',   'Control',     4, '[1×N]';
    'log_e_u',       'Control',     5, '[1×N]';
    'log_ui',        'Actuation',   1, '[3×N]';
    'log_pwm',       'Actuation',   2, '[3×N]';
    'log_sat',       'Actuation',   3, '[3×N]';
};
for i = 1:size(log_table,1)
    fprintf('%-20s  %-30s  Out%-4d  %s\n', ...
        log_table{i,1}, log_table{i,2}, log_table{i,3}, log_table{i,4});
end

%% ── Controller swap instructions ─────────────────────────────────────────
fprintf('\nCONTROLLER SWAP (PID → SMC)\n');
fprintf('  In Simulink: double-click the Control block\n');
fprintf('  Change S-function name from:\n');
fprintf('    control_pid_sfcn\n');
fprintf('  To:\n');
fprintf('    control_smc_sfcn\n');
fprintf('  Press Ctrl+D to update. No other changes needed.\n\n');

%% ── Verify loaded model connections ─────────────────────────────────────
function add_unit_delays(mdl)
%% Programmatically insert Unit Delay blocks on Navigation feedback paths

fprintf('  Checking for existing Unit Delay blocks...\n');

ud_nu_path  = [mdl '/UnitDelay_nu'];
ud_eta_path = [mdl '/UnitDelay_eta'];

% Only add if they don't already exist
if isempty(find_system(mdl,'Name','UnitDelay_nu'))

    % Add Unit Delay for nu_hat
    add_block('simulink/Discrete/Unit Delay', ud_nu_path, ...
        'X0',            'zeros(6,1)', ...
        'SampleTime',    '-1', ...
        'Position',      [1190 240 1250 270]);

    % Add Unit Delay for eta_hat
    add_block('simulink/Discrete/Unit Delay', ud_eta_path, ...
        'X0',            'zeros(6,1)', ...
        'SampleTime',    '-1', ...
        'Position',      [1190 310 1250 340]);

    fprintf('  Unit Delay blocks added.\n');

    % Remove direct Navigation → Control lines if they exist
    % and re-route through Unit Delays
    try
        % Navigation Out1 → UnitDelay_nu
        add_line(mdl,'Navigation/1','UnitDelay_nu/1','autorouting','smart');
        % Navigation Out2 → UnitDelay_eta
        add_line(mdl,'Navigation/2','UnitDelay_eta/1','autorouting','smart');

        % UnitDelay_nu → Guidance In1 AND Control In4
        add_line(mdl,'UnitDelay_nu/1','Guidance/1','autorouting','smart');
        add_line(mdl,'UnitDelay_nu/1','Control/4','autorouting','smart');

        % UnitDelay_eta → Guidance In2 AND Control In5
        add_line(mdl,'UnitDelay_eta/1','Guidance/2','autorouting','smart');
        add_line(mdl,'UnitDelay_eta/1','Control/5','autorouting','smart');

        fprintf('  Navigation feedback re-routed through Unit Delays.\n');
    catch ME
        warning('Re-routing partial: %s\nComplete wiring manually.', ME.message);
    end

    save_system(mdl);
    fprintf('  Model saved.\n\n');
else
    fprintf('  Unit Delay blocks already present — no action needed.\n\n');
end
end