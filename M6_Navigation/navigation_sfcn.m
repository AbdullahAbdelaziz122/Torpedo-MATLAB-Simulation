function navigation_sfcn(block)
%% navigation_sfcn.m  —  Level-2 S-Function: Navigation Module (M6)
%
% PURPOSE:
%   Implements the Navigation subsystem as a Level-2 S-Function.
%   Phase 4: perfect pass-through observer.
%   Future phases: replace nav_passthrough with EKF by modifying
%   ONLY this file — all downstream modules are unaffected.
%
% BLOCK PORTS:
%   Inputs:
%     u1 : x_true  [12×1]  — true state from Dynamics integrator output
%
%   Outputs:
%     y1 : nu_hat  [6×1]   — estimated body velocities [u v w p q r]
%     y2 : eta_hat [6×1]   — estimated position + angles [x y z phi th psi]
%     y3 : speed   [1×1]   — estimated scalar speed |nu_hat| (m/s)
%
% DIRECT FEEDTHROUGH:
%   true — outputs depend directly on the current input state.
%
% DESIGN NOTE — Why a separate Navigation block matters even in Phase 4:
%   Wiring all controllers to read from y1/y2/y3 here (not directly from
%   Dynamics) means that when you upgrade to a noisy sensor model or EKF
%   in a future phase, ZERO rewiring is required in the Control or Guidance
%   subsystems. The contract is established now at zero extra cost.
%
% AUTHOR: AUV Simulation Project — Phase 4

setup(block);

% =========================================================================
function setup(block)

block.NumInputPorts  = 1;
block.NumOutputPorts = 3;

% u1: x_true [12×1]
block.InputPort(1).Dimensions  = 12;
block.InputPort(1).DatatypeID  = 0;    % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = true;

% y1: nu_hat [6×1]
block.OutputPort(1).Dimensions = 6;
block.OutputPort(1).DatatypeID = 0;
block.OutputPort(1).Complexity = 'Real';

% y2: eta_hat [6×1]
block.OutputPort(2).Dimensions = 6;
block.OutputPort(2).DatatypeID = 0;
block.OutputPort(2).Complexity = 'Real';

% y3: speed [scalar]
block.OutputPort(3).Dimensions = 1;
block.OutputPort(3).DatatypeID = 0;
block.OutputPort(3).Complexity = 'Real';

block.NumContStates      = 0;
block.SampleTimes        = [-1 0];   % inherited
block.SimStateCompliance = 'DefaultSimState';

block.RegBlockMethod('InitializeConditions', @InitConditions);
block.RegBlockMethod('Outputs',              @Outputs);
block.RegBlockMethod('Terminate',            @Terminate);

% =========================================================================
function InitConditions(block) %#ok<INUSD>

% =========================================================================
function Outputs(block)

x_true = block.InputPort(1).Data;

try
    auv = evalin('base', 'auv');
catch
    error('navigation_sfcn: auv struct not found. Run auv_params.m.');
end

% Phase 4: perfect pass-through
% To upgrade: replace this call with an EKF function
nav = nav_passthrough(x_true, auv);

block.OutputPort(1).Data = nav.nu_hat;
block.OutputPort(2).Data = nav.eta_hat;
block.OutputPort(3).Data = nav.speed;

% =========================================================================
function Terminate(block) %#ok<INUSD>

% =========================================================================
% Embedded copies — canonical source is navigation_lib.m
% =========================================================================

function nav = nav_passthrough(x_true, auv)
nu_true  = x_true(1:6);
eta_true = x_true(7:12);

sigma_v = auv.nav.sigma_vel;   sigma_p = auv.nav.sigma_pos;
sigma_a = auv.nav.sigma_angle; sigma_r = auv.nav.sigma_rate;

nu_hat  = nu_true  + noise6([sigma_v;sigma_v;sigma_v;sigma_r;sigma_r;sigma_r]);
eta_hat = eta_true + noise6([sigma_p;sigma_p;sigma_p;sigma_a;sigma_a;sigma_a]);
eta_hat = wrap_eta(eta_hat);

nav.nu_hat  = nu_hat;
nav.eta_hat = eta_hat;
nav.speed   = sqrt(nu_hat(1)^2 + nu_hat(2)^2 + nu_hat(3)^2);

function n = noise6(sigma_vec)
if all(sigma_vec == 0),  n = zeros(6,1);
else,                     n = sigma_vec .* randn(6,1);
end

function eta_w = wrap_eta(eta)
eta_w    = eta;
eta_w(4) = atan2(sin(eta(4)), cos(eta(4)));
eta_w(5) = atan2(sin(eta(5)), cos(eta(5)));
eta_w(6) = atan2(sin(eta(6)), cos(eta(6)));
