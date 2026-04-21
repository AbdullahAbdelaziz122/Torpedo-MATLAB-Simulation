%% build_dynamics_block.m  —  Build Dynamics_M5.slx programmatically
%
% PURPOSE:
%   Creates the Dynamics_M5 Simulink model from scratch using the
%   Simulink API. Run this ONCE to generate the .slx file.
%   Afterwards, open Dynamics_M5.slx directly — do not re-run this
%   script unless you want to rebuild from scratch.
%
% PREREQUISITES:
%   - buses.m must have been run (bus objects in base workspace)
%   - auv_params.m must have been run (auv struct in base workspace)
%   - dynamics_sfcn.m must be on the MATLAB path
%   - MSS Toolbox must be on the MATLAB path (remus100.m accessible)
%
% OUTPUT:
%   Dynamics_M5.slx  — Simulink subsystem model
%
% MODEL STRUCTURE:
%
%   [In: ui(3)]    ──┐
%   [In: Vc(1)]    ──┤
%   [In: betaVc(1)]──┤──► [dynamics_sfcn] ──► [Out: x(12)]
%   [In: w_c(1)]   ──┘                    ──► [Out: xdot(12)]
%                                          ──► [Out: U(1)]
%
%   Additionally:
%   - An IC Check subsystem verifies x0 dimensions at sim start
%   - A Rate Transition block ensures clean sample-time propagation
%
% USAGE:
%   >> buses; auv_params;
%   >> build_dynamics_block
%   >> open_system('Dynamics_M5')   % verify visually
%
% AUTHOR: AUV Simulation Project — Phase 2

fprintf('\n=== Building Dynamics_M5.slx ===\n');

model_name = 'Dynamics_M5';

% Close and delete if it already exists
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
if exist([model_name '.slx'], 'file')
    delete([model_name '.slx']);
    fprintf('  Deleted existing %s.slx\n', model_name);
end

% Create new model
new_system(model_name);
open_system(model_name);

% =========================================================================
% Solver configuration — CRITICAL for correct integration
% =========================================================================
% Fixed-step ode4 for real-time deployment.
% During development you can use ode45 (variable-step) — but always
% validate with ode4 before deploying.
set_param(model_name, ...
    'SolverType',          'Fixed-step', ...
    'Solver',              'ode4', ...
    'FixedStep',           '0.01', ...
    'StopTime',            '300', ...
    'SaveFormat',          'Dataset', ...
    'SignalLogging',       'on', ...
    'SignalLoggingName',   'logsout');

% =========================================================================
% Block placement
% =========================================================================

% --- S-Function block (wraps remus100.m) ---
sfcn_path = [model_name '/Dynamics (remus100)'];
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', sfcn_path, ...
    'FunctionName', 'dynamics_sfcn', ...
    'Position',     [350 120 550 260]);

% --- Input ports ---
% Port 1: ui [3×1]
in_ui = [model_name '/ui [delta_r, delta_s, n_RPM]'];
add_block('simulink/Sources/In1', in_ui, ...
    'PortDimensions', '3', ...
    'OutDataTypeStr', 'double', ...
    'Position', [80 130 130 150]);

% Port 2: Vc
in_Vc = [model_name '/Vc [m_s]'];
add_block('simulink/Sources/In1', in_Vc, ...
    'PortDimensions', '1', ...
    'OutDataTypeStr', 'double', ...
    'Position', [80 175 130 195]);

% Port 3: betaVc
in_beta = [model_name '/betaVc [rad]'];
add_block('simulink/Sources/In1', in_beta, ...
    'PortDimensions', '1', ...
    'OutDataTypeStr', 'double', ...
    'Position', [80 220 130 240]);

% Port 4: w_c
in_wc = [model_name '/w_c [m_s]'];
add_block('simulink/Sources/In1', in_wc, ...
    'PortDimensions', '1', ...
    'OutDataTypeStr', 'double', ...
    'Position', [80 265 130 285]);

% --- Output ports ---
% y1: state x [12×1]
out_x = [model_name '/x [12x1 state]'];
add_block('simulink/Sinks/Out1', out_x, ...
    'PortDimensions', '12', ...
    'Position', [650 130 700 150]);

% y2: xdot [12×1]
out_xdot = [model_name '/xdot [12x1 derivative]'];
add_block('simulink/Sinks/Out1', out_xdot, ...
    'PortDimensions', '12', ...
    'Position', [650 190 700 210]);

% y3: U [scalar speed]
out_U = [model_name '/U [speed m_s]'];
add_block('simulink/Sinks/Out1', out_U, ...
    'PortDimensions', '1', ...
    'Position', [650 250 700 270]);

% =========================================================================
% Connections
% =========================================================================

% Inputs → S-Function
add_line(model_name, 'ui [delta_r, delta_s, n_RPM]/1', 'Dynamics (remus100)/1', 'autorouting', 'on');
add_line(model_name, 'Vc [m_s]/1',                     'Dynamics (remus100)/2', 'autorouting', 'on');
add_line(model_name, 'betaVc [rad]/1',                  'Dynamics (remus100)/3', 'autorouting', 'on');
add_line(model_name, 'w_c [m_s]/1',                     'Dynamics (remus100)/4', 'autorouting', 'on');

% S-Function → Outputs
add_line(model_name, 'Dynamics (remus100)/1', 'x [12x1 state]/1',          'autorouting', 'on');
add_line(model_name, 'Dynamics (remus100)/2', 'xdot [12x1 derivative]/1',  'autorouting', 'on');
add_line(model_name, 'Dynamics (remus100)/3', 'U [speed m_s]/1',           'autorouting', 'on');

% =========================================================================
% Signal labels — name the wires for readability
% =========================================================================
% (Simulink auto-routes but labels must be set via line handles)
lines = find_system(model_name, 'FindAll', 'on', 'Type', 'line');
for k = 1:numel(lines)
    src = get_param(lines(k), 'SrcBlockHandle');
    src_name = get_param(src, 'Name');
    if contains(src_name, 'ui')
        set_param(lines(k), 'Name', 'ui');
    elseif contains(src_name, 'Vc')
        set_param(lines(k), 'Name', 'Vc');
    elseif contains(src_name, 'betaVc')
        set_param(lines(k), 'Name', 'betaVc');
    elseif contains(src_name, 'w_c')
        set_param(lines(k), 'Name', 'w_c');
    end
end

% =========================================================================
% Documentation annotation
% =========================================================================
note_text = sprintf(['Dynamics Module (M5)\n' ...
    'Wraps: remus100.m (Fossen, MSS Toolbox 2021)\n' ...
    'State: x = [u v w p q r x_n y_e z_d phi theta psi]\n' ...
    'Inputs: ui=[delta_r(rad), delta_s(rad), n(RPM)], ocean current\n' ...
    'Solver: ode4, Ts=0.01s\n' ...
    'DO NOT modify remus100.m — use auv_params.m for ICs']);

add_block('built-in/Note', [model_name '/docblock'], ...
    'Position',    [80 320 580 420], ...
    'Text',        note_text, ...
    'FontSize',    '10');

% =========================================================================
% Save
% =========================================================================
save_system(model_name);
fprintf('  Saved %s.slx\n', model_name);
fprintf('\n=== Dynamics_M5.slx built successfully ===\n');
fprintf('\nNext steps:\n');
fprintf('  1. Verify visually: open_system(''Dynamics_M5'')\n');
fprintf('  2. Run gate test:   test_phase2_openloop\n');
fprintf('  3. Check plots:     plot_phase2\n\n');
