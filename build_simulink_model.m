%% build_simulink_model.m  —  Build the complete AUV Simulink model
%
% PURPOSE:
%   Programmatically creates AUV_TopLevel.slx by wiring all 7 S-Function
%   modules together. Run once to generate the model; open it afterwards
%   for inspection, simulation, and real-time deployment.
%
% PREREQUISITES:
%   buses.m and auv_params.m must have been run (bus objects in workspace).
%   All _sfcn.m files must be on the MATLAB path.
%   MSS Toolbox must be on path (remus100.m accessible).
%
% MODULE PORT MAP (verified from S-Function source):
%
%   DYNAMICS (dynamics_sfcn)
%     In1: ui      [3×1]  — actuator commands
%     In2: Vc      [1×1]  — current speed
%     In3: betaVc  [1×1]  — current direction
%     In4: w_c     [1×1]  — vertical current
%     Out1: x      [12×1] — true state
%     Out2: xdot   [12×1] — state derivative
%     Out3: U      [1×1]  — speed scalar
%
%   NAVIGATION (navigation_sfcn)
%     In1:  x_true [12×1]
%     Out1: nu_hat  [6×1]
%     Out2: eta_hat [6×1]
%     Out3: speed   [1×1]
%
%   GUIDANCE (guidance_los_sfcn)
%     In1: nu_hat    [6×1]
%     In2: eta_hat   [6×1]
%     In3: pt        [3×1]  — path point
%     In4: pt_vel    [3×1]  — path velocity
%     In5: desire_spd[3×1]  — nominal speed vector
%     Out1: chi_d    [1×1]
%     Out2: upsilon_d[1×1]
%     Out3: ud       [1×1]
%     Out4: chi_v    [1×1]  — logging
%     Out5: upsilon_v[1×1]  — logging
%     Out6: los_err  [3×1]  — logging
%
%   CONTROL (control_pid_sfcn)
%     In1: chi_d     [1×1]
%     In2: upsilon_d [1×1]
%     In3: ud        [1×1]
%     In4: nu_hat    [6×1]
%     In5: eta_hat   [6×1]
%     Out1: tau_ctrl [6×1]
%     Out2: n_direct [1×1]
%     Out3: e_chi    [1×1]  — logging
%     Out4: e_theta  [1×1]  — logging
%     Out5: e_u      [1×1]  — logging
%
%   ACTUATION (actuation_sfcn)
%     In1: tau_ctrl  [6×1]
%     In2: U         [1×1]  — speed from dynamics
%     In3: n_direct  [1×1]
%     Out1: ui       [3×1]
%     Out2: pwm      [3×1]
%     Out3: sat_flags[3×1]  uint8
%
%   ENVIRONMENT (MATLAB Function block)
%     Outputs: Vc, betaVc, w_c, tau_env[6×1]
%
%   PATH GENERATOR (MATLAB Function block)
%     Input:  t (clock)
%     Outputs: pt[3×1], pt_vel[3×1]
%
% USAGE:
%   >> buses; auv_params; auv_params_env_patch;
%   >> build_simulink_model
%   >> open_system('AUV_TopLevel')
%
% AUTHOR: AUV Simulation Project — Simulink Wiring

fprintf('\n=== Building AUV_TopLevel.slx ===\n');

%% ── 0. Prerequisites check ───────────────────────────────────────────────
if ~exist('auv','var')
    error('Run buses; auv_params; auv_params_env_patch first.');
end
if isempty(which('remus100'))
    error('MSS Toolbox not on path. Run mssstart.m first.');
end

%% ── 1. Create / reset model ──────────────────────────────────────────────
mdl = 'AUV_TopLevel';
if bdIsLoaded(mdl), close_system(mdl,0); end
if exist([mdl '.slx'],'file'), delete([mdl '.slx']); end

new_system(mdl);
open_system(mdl);

%% ── 2. Solver and model settings ─────────────────────────────────────────
set_param(mdl, ...
    'SolverType',           'Fixed-step', ...
    'Solver',               'ode4', ...
    'FixedStep',            num2str(auv.sim.Ts), ...
    'StopTime',             num2str(auv.sim.T_end), ...
    'SignalLogging',        'on', ...
    'SignalLoggingName',    'logsout', ...
    'SaveFormat',           'Dataset');

%% ── 3. Layout constants ───────────────────────────────────────────────────
% Block positions: [left top right bottom]
% Columns (X centres): Env=120, Path=120, Guidance=320, Control=520, Actuation=720, Dynamics=920, Navigation=1120
% Rows (Y centres):    top=80, main=260, bottom=440

W = 140; H = 80;   % standard block width / height
col = struct('env',80,'path',80,'guid',290,'ctrl',490,'act',690,'dyn',890,'nav',1090,'log',1290);
row = struct('env',60,'path',200,'main',240,'nav',420,'log',600);

%% ── 4. Add all S-Function blocks ─────────────────────────────────────────
fprintf('  Adding S-Function blocks...\n');

% Helper: add an S-Function block
add_sfcn = @(path, name, pos) add_block( ...
    'simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    [mdl '/' name], ...
    'FunctionName', path, ...
    'Position',     pos);

% M5 — Dynamics
b_dyn = [col.dyn,         row.main,      col.dyn+W,      row.main+H];
add_sfcn('dynamics_sfcn',    'Dynamics',   b_dyn);

% M6 — Navigation
b_nav = [col.nav,         row.nav,       col.nav+W,      row.nav+H];
add_sfcn('navigation_sfcn',  'Navigation', b_nav);

% M2 — Guidance
b_guid = [col.guid,       row.main-60,   col.guid+W,     row.main-60+H];
add_sfcn('guidance_los_sfcn','Guidance',   b_guid);

% M3 — Control
b_ctrl = [col.ctrl,       row.main,      col.ctrl+W,     row.main+H];
add_sfcn('control_pid_sfcn', 'Control',   b_ctrl);

% M4 — Actuation
b_act = [col.act,         row.main,      col.act+W,      row.main+H];
add_sfcn('actuation_sfcn',   'Actuation', b_act);

fprintf('    S-Function blocks added.\n');

%% ── 5. Environment block (MATLAB Function) ───────────────────────────────
fprintf('  Adding Environment block...\n');

b_env = [col.env, row.env, col.env+W, row.env+H];
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl '/Environment'], 'Position', b_env);

% Set the MATLAB function code
env_code = [...
    'function [Vc, betaVc, w_c, tau_env] = Environment(t)\n'...
    '%% Reads environment parameters from auv struct in base workspace.\n'...
    'auv = evalin(''base'',''auv'');\n'...
    'Vc     = auv.env.Vc_mean;\n'...
    'betaVc = auv.env.betaVc_mean;\n'...
    'w_c    = auv.env.w_c;\n'...
    'tau_env = zeros(6,1);\n'];

%set_param([mdl '/Environment'], 'MATLABFunctionConfiguration', '');
% Note: MATLAB Function block code is set via the editor;
% we set it programmatically via the script object
try
    rt = sfroot();
    m  = rt.find('-isa','Simulink.BlockDiagram','Name', mdl);
    eb = m.find('-isa','Stateflow.EMChart','Path',[mdl '/Environment']);
    eb.Script = sprintf(env_code);
catch
    warning('Environment function body must be set manually — open the block and paste the code from environment_lib.m');
end

%% ── 6. Path Generator block (MATLAB Function) ────────────────────────────
fprintf('  Adding Path Generator block...\n');

b_path = [col.path, row.path, col.path+W, row.path+H];
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl '/PathGenerator'], 'Position', b_path);

path_code = [...
    'function [pt, pt_vel] = PathGenerator(t)\n'...
    '%% Queries the helix path at time t.\n'...
    'persistent path_data\n'...
    'if isempty(path_data)\n'...
    '    auv = evalin(''base'',''auv'');\n'...
    '    m=0.6; dt=auv.sim.Ts; T=auv.sim.T_end;\n'...
    '    time=0:dt:T; Yr=m*time;\n'...
    '    X=60*cos(0.02618*Yr); Y=60*sin(0.02618*Yr); Z=2+(2*Yr/200);\n'...
    '    dX=-60*0.02618*m*sin(0.02618*Yr);\n'...
    '    dY= 60*0.02618*m*cos(0.02618*Yr);\n'...
    '    dZ= 2*m/200*ones(size(time));\n'...
    '    path_data.pts=[X;Y;Z]; path_data.vel=[dX;dY;dZ];\n'...
    '    path_data.t_vec=time; path_data.T=T; path_data.dt=dt;\n'...
    'end\n'...
    't_c = max(0, min(t, path_data.T));\n'...
    'idx = max(1, min(round(t_c/path_data.dt)+1, size(path_data.pts,2)));\n'...
    'pt     = path_data.pts(:,idx);\n'...
    'pt_vel = path_data.vel(:,idx);\n'];

try
    rt = sfroot();
    m  = rt.find('-isa','Simulink.BlockDiagram','Name', mdl);
    pb = m.find('-isa','Stateflow.EMChart','Path',[mdl '/PathGenerator']);
    pb.Script = sprintf(path_code);
catch
    warning('PathGenerator function body must be set manually.');
end

%% ── 7. Clock block (time source for path and environment) ────────────────
add_block('simulink/Sources/Clock', [mdl '/Clock'], ...
    'Position', [col.env-80, row.env+20, col.env-40, row.env+50]);

%% ── 8. Desired speed constant ────────────────────────────────────────────
add_block('simulink/Sources/Constant', [mdl '/DesiredSpeed'], ...
    'Value',    '[auv.guid.U_min; 0; 0]', ...
    'Position', [col.guid-160, row.main+20, col.guid-80, row.main+50]);

%% ── 9. To-Workspace logging blocks ──────────────────────────────────────
fprintf('  Adding logging blocks...\n');

log_y = row.log;
log_blocks = { ...
    'log_x',        [mdl '/Dynamics'],   1, [col.dyn+W+20,   log_y,   col.dyn+W+120,  log_y+30]; ...
    'log_nu_hat',   [mdl '/Navigation'], 1, [col.nav+W+20,   log_y,   col.nav+W+120,  log_y+30]; ...
    'log_eta_hat',  [mdl '/Navigation'], 2, [col.nav+W+20,   log_y+40,col.nav+W+120,  log_y+70]; ...
    'log_tau_ctrl', [mdl '/Control'],    1, [col.ctrl+W+20,  log_y,   col.ctrl+W+120, log_y+30]; ...
    'log_n_direct', [mdl '/Control'],    2, [col.ctrl+W+20,  log_y+40,col.ctrl+W+120, log_y+70]; ...
    'log_ui',       [mdl '/Actuation'],  1, [col.act+W+20,   log_y,   col.act+W+120,  log_y+30]; ...
    'log_pwm',      [mdl '/Actuation'],  2, [col.act+W+20,   log_y+40,col.act+W+120,  log_y+70]; ...
};

for i = 1:size(log_blocks,1)
    var_name = log_blocks{i,1};
    bpos     = log_blocks{i,4};
    add_block('simulink/Sinks/To Workspace', [mdl '/' var_name], ...
        'VariableName', var_name, ...
        'SaveFormat',   'Array', ...
        'Position',     bpos);
end

%% ── 10. Wire all connections ─────────────────────────────────────────────
fprintf('  Wiring connections...\n');

% Helper
wire = @(src_blk, src_port, dst_blk, dst_port) ...
    add_line(mdl, ...
        [src_blk '/' num2str(src_port)], ...
        [dst_blk '/' num2str(dst_port)], ...
        'autorouting','smart');

%% ── Signal flow: the main closed loop ───────────────────────────────────

% Clock → Environment (time input)
wire('Clock',         1, 'Environment',   1);
% Clock → PathGenerator (time input)
wire('Clock',         1, 'PathGenerator', 1);

% Environment → Dynamics (current params)
wire('Environment',   1, 'Dynamics',      2);   % Vc
wire('Environment',   2, 'Dynamics',      3);   % betaVc
wire('Environment',   3, 'Dynamics',      4);   % w_c

% Dynamics → Navigation
wire('Dynamics',      1, 'Navigation',    1);   % x[12×1]

% Navigation → Guidance
wire('Navigation',    1, 'Guidance',      1);   % nu_hat[6×1]
wire('Navigation',    2, 'Guidance',      2);   % eta_hat[6×1]

% PathGenerator → Guidance
wire('PathGenerator', 1, 'Guidance',      3);   % pt[3×1]
wire('PathGenerator', 2, 'Guidance',      4);   % pt_vel[3×1]

% DesiredSpeed → Guidance
wire('DesiredSpeed',  1, 'Guidance',      5);   % desire_spd[3×1]

% Guidance → Control
wire('Guidance',      1, 'Control',       1);   % chi_d
wire('Guidance',      2, 'Control',       2);   % upsilon_d
wire('Guidance',      3, 'Control',       3);   % ud

% Navigation → Control (state feedback)
wire('Navigation',    1, 'Control',       4);   % nu_hat[6×1]
wire('Navigation',    2, 'Control',       5);   % eta_hat[6×1]

% Control → Actuation
wire('Control',       1, 'Actuation',     1);   % tau_ctrl[6×1]
wire('Control',       2, 'Actuation',     3);   % n_direct[1×1]

% Dynamics speed → Actuation (fin effectiveness depends on speed)
wire('Dynamics',      3, 'Actuation',     2);   % U[1×1]

% Actuation → Dynamics (close the loop)
wire('Actuation',     1, 'Dynamics',      1);   % ui[3×1]

fprintf('    Main loop wired.\n');

%% ── Logging connections ──────────────────────────────────────────────────
wire('Dynamics',   1, 'log_x',        1);
wire('Navigation', 1, 'log_nu_hat',   1);
wire('Navigation', 2, 'log_eta_hat',  1);
wire('Control',    1, 'log_tau_ctrl', 1);
wire('Control',    2, 'log_n_direct', 1);
wire('Actuation',  1, 'log_ui',       1);
wire('Actuation',  2, 'log_pwm',      1);

fprintf('    Logging connections wired.\n');

%% ── 11. Block labels (signal names on wires) ─────────────────────────────
fprintf('  Setting signal labels...\n');

% Label the key wires by finding line handles and setting names
try
    lines = find_system(mdl,'FindAll','on','Type','line');
    for i = 1:numel(lines)
        src_h  = get_param(lines(i),'SrcBlockHandle');
        src_p  = get_param(lines(i),'SrcPortIndex');
        src_nm = get_param(src_h,'Name');
        switch src_nm
            case 'Dynamics'
                names = {'x[12]','xdot[12]','U'};
                if src_p <= numel(names), set_param(lines(i),'Name',names{src_p}); end
            case 'Navigation'
                names = {'nu_hat[6]','eta_hat[6]','speed'};
                if src_p <= numel(names), set_param(lines(i),'Name',names{src_p}); end
            case 'Guidance'
                names = {'chi_d','upsilon_d','ud','chi_v','upsilon_v','los_err[3]'};
                if src_p <= numel(names), set_param(lines(i),'Name',names{src_p}); end
            case 'Control'
                names = {'tau_ctrl[6]','n_direct','e_chi','e_theta','e_u'};
                if src_p <= numel(names), set_param(lines(i),'Name',names{src_p}); end
            case 'Actuation'
                names = {'ui[3]','pwm[3]','sat_flags[3]'};
                if src_p <= numel(names), set_param(lines(i),'Name',names{src_p}); end
        end
    end
catch ME
    warning('Signal labelling partial: %s', ME.message);
end

%% ── 12. Scope blocks for live monitoring ─────────────────────────────────
fprintf('  Adding scope blocks...\n');

scope_y = row.main - 160;

add_block('simulink/Sinks/Scope', [mdl '/Scope_States'], ...
    'NumInputPorts', '3', ...
    'Position', [col.nav+W+160, scope_y, col.nav+W+230, scope_y+60]);

wire('Dynamics',   1, 'Scope_States', 1);   % full state
wire('Navigation', 1, 'Scope_States', 2);   % nu_hat
wire('Navigation', 2, 'Scope_States', 3);   % eta_hat

add_block('simulink/Sinks/Scope', [mdl '/Scope_Control'], ...
    'NumInputPorts', '2', ...
    'Position', [col.ctrl+W+160, scope_y, col.ctrl+W+230, scope_y+60]);

wire('Control',   1, 'Scope_Control', 1);   % tau_ctrl
wire('Actuation', 1, 'Scope_Control', 2);   % ui

%% ── 13. Model annotation ─────────────────────────────────────────────────
note = sprintf(['AUV Simulation — Complete 7-Module Architecture\n'...
    'Dynamics: remus100.m (Fossen MSS Toolbox)\n'...
    'Control:  PID (swap to SMC: change S-Function name to control_smc_sfcn)\n'...
    'Guidance: 3D LOS\n'...
    'Solver:   ode4 fixed-step, Ts=%.3f s'], auv.sim.Ts);

add_block('built-in/Note', [mdl '/ModelNote'], ...
    'Position', [20, 20, 600, 50], ...
    'Text', note, 'FontSize', '9');

%% ── 14. Auto-arrange and save ────────────────────────────────────────────
Simulink.BlockDiagram.arrangeSystem(mdl);
save_system(mdl);

fprintf('\n=== AUV_TopLevel.slx built successfully ===\n\n');
fprintf('NEXT STEPS:\n');
fprintf('  1. open_system(''AUV_TopLevel'')  — inspect wiring visually\n');
fprintf('  2. Check Environment and PathGenerator MATLAB Function blocks\n');
fprintf('     (open each, paste code from environment_lib.m / path_lib.m)\n');
fprintf('  3. sim(''AUV_TopLevel'')          — run the simulation\n');
fprintf('  4. To swap controller:            change S-Function name in\n');
fprintf('     Control block from control_pid_sfcn → control_smc_sfcn\n\n');

%% ── Helper: check for algebraic loops ────────────────────────────────────
fprintf('Checking for algebraic loops (may take a moment)...\n');
try
    set_param(mdl,'AlgebraicLoopMsg','error');
    % Compile-only check
    feval(mdl,[],[],[],'compile');
    feval(mdl,[],[],[],'term');
    fprintf('  No algebraic loops detected.\n\n');
catch ME
    fprintf('  Warning: %s\n', ME.message);
    fprintf('  If an algebraic loop is reported, add a Unit Delay on\n');
    fprintf('  the Navigation→Control feedback path.\n\n');
end
