function dynamics_sfcn(block)
%% dynamics_sfcn.m  —  Level-2 MATLAB S-Function: REMUS 100 Dynamics
%
% PURPOSE:
%   Wraps remus100.m (Fossen, MSS Toolbox) as a Simulink Level-2
%   S-Function block. This is the ONLY place remus100.m is called.
%   No other module may call it directly.
%
% BLOCK PORTS:
%   Inputs:
%     u1 : ui      [3×1]  — actuator commands [delta_r(rad), delta_s(rad), n(RPM)]
%     u2 : Vc      [1×1]  — current speed magnitude (m/s)
%     u3 : betaVc  [1×1]  — current direction NED (rad)
%     u4 : w_c     [1×1]  — vertical current speed (m/s)
%
%   Outputs:
%     y1 : x       [12×1] — full state [u v w p q r x_n y_e z_d phi theta psi]
%     y2 : xdot    [12×1] — state derivative (for logging and observer)
%     y3 : U       [1×1]  — vehicle speed scalar |nu| (m/s)
%
%   Continuous States:
%     12 states — integrated by Simulink's ODE solver
%
% USAGE IN SIMULINK:
%   Add an S-Function block, set S-function name to 'dynamics_sfcn'.
%   No S-function parameters needed — all parameters come from the
%   auv struct in base workspace.
%
% DESIGN NOTES:
%   - Uses continuous states so Simulink's solver handles integration.
%     This is correct for real-time (ode4 fixed-step) and desktop (ode45).
%   - remus100.m applies its own internal saturation on ui — do NOT
%     pre-saturate inputs before this block (double saturation distorts
%     the dynamics near limits).
%   - The block reads initial conditions from auv.ic.x0 in the base
%     workspace. Change ICs by modifying auv_params.m, not here.
%   - xdot output (y2) is the raw derivative BEFORE integration — useful
%     for the Navigation observer and for logging accelerations.
%
% AUTHOR: AUV Simulation Project — Phase 2
% REQUIRES: MSS Toolbox (remus100.m, spheroid.m, imlay61.m, etc.)

setup(block);

% =========================================================================
function setup(block)

% --- Register number of ports ---
block.NumInputPorts  = 4;
block.NumOutputPorts = 3;

% --- Input port sizes and types ---
% u1: ui [3×1] actuator commands
block.InputPort(1).Dimensions  = 3;
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = true;  % state output, not feedthrough

% u2: Vc [scalar]
block.InputPort(2).Dimensions  = 1;
block.InputPort(2).DatatypeID  = 0;
block.InputPort(2).Complexity  = 'Real';
block.InputPort(2).DirectFeedthrough = true;

% u3: betaVc [scalar]
block.InputPort(3).Dimensions  = 1;
block.InputPort(3).DatatypeID  = 0;
block.InputPort(3).Complexity  = 'Real';
block.InputPort(3).DirectFeedthrough = true;

% u4: w_c [scalar]
block.InputPort(4).Dimensions  = 1;
block.InputPort(4).DatatypeID  = 0;
block.InputPort(4).Complexity  = 'Real';
block.InputPort(4).DirectFeedthrough = true;

% --- Output port sizes ---
% y1: full state x [12×1]
block.OutputPort(1).Dimensions = 12;
block.OutputPort(1).DatatypeID = 0;
block.OutputPort(1).Complexity = 'Real';

% y2: state derivative xdot [12×1]
block.OutputPort(2).Dimensions = 12;
block.OutputPort(2).DatatypeID = 0;
block.OutputPort(2).Complexity = 'Real';

% y3: speed scalar U [1×1]
block.OutputPort(3).Dimensions = 1;
block.OutputPort(3).DatatypeID = 0;
block.OutputPort(3).Complexity = 'Real';

% --- Continuous states (12) ---
block.NumContStates = 12;

% --- Sample times: inherited (continuous) ---
block.SampleTimes = [0 0];  % continuous

% --- Specify that the block is not a source (requires input to run) ---
block.SimStateCompliance = 'DefaultSimState';

% --- Register callbacks ---
block.RegBlockMethod('InitializeConditions', @InitConditions);
block.RegBlockMethod('Outputs',              @Outputs);
block.RegBlockMethod('Derivatives',          @Derivatives);
block.RegBlockMethod('Terminate',            @Terminate);

% =========================================================================
function InitConditions(block)
% Called once at simulation start. Reads IC from auv.ic.x0 in base workspace.

try
    auv = evalin('base', 'auv');
    x0  = auv.ic.x0;
    if numel(x0) ~= 12
        error('auv.ic.x0 must be 12×1. Run auv_params.m first.');
    end
catch ME
    warning('dynamics_sfcn: Could not read auv.ic.x0. Using zeros. Error: %s', ME.message);
    x0 = zeros(12, 1);
end

block.ContStates.Data = x0(:);

% =========================================================================
function Outputs(block)
% Called at each output step. State is already integrated — just assign outputs.

x = block.ContStates.Data;

% Compute xdot and U for outputs (remus100 is cheap to call twice;
% Simulink calls Derivatives separately for the solver steps)
ui     = block.InputPort(1).Data;
Vc     = block.InputPort(2).Data;
betaVc = block.InputPort(3).Data;
w_c    = block.InputPort(4).Data;

[xdot, U] = remus100(x, ui, Vc, betaVc, w_c);

block.OutputPort(1).Data = x;       % y1: state
block.OutputPort(2).Data = xdot;    % y2: derivative
block.OutputPort(3).Data = U;       % y3: speed

% =========================================================================
function Derivatives(block)
% Called by the ODE solver at each integration step.
% This is where the actual integration happens — Simulink advances
% ContStates.Data using the derivative returned here.

x      = block.ContStates.Data;
ui     = block.InputPort(1).Data;
Vc     = block.InputPort(2).Data;
betaVc = block.InputPort(3).Data;
w_c    = block.InputPort(4).Data;

xdot = remus100(x, ui, Vc, betaVc, w_c);

block.Derivatives.Data = xdot;

% =========================================================================
function Terminate(block) %#ok<INUSD>
% Nothing to clean up.
