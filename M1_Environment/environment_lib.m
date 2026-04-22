%% environment_lib.m  —  Environment Module Function Library  (Phase 7)
%
% PURPOSE:
%   Generates all environmental inputs consumed by the Dynamics module:
%     - Ocean current parameters (Vc, betaVc, w_c) → fed directly to remus100.m
%     - Wave disturbance forces tau_wave [6×1]       → added to tau before remus100.m
%     - Wind forces tau_wind [6×1]                   → added to tau (Phase 7: zero)
%
% ARCHITECTURE:
%   This module outputs EnvBus signals. The Dynamics S-Function reads
%   Vc/betaVc/w_c from EnvBus and passes them to remus100.m as arguments.
%   tau_env = tau_wave + tau_wind is added to tau_ctrl BEFORE the
%   dynamics call — this is the force-superposition principle from Fossen.
%
%   remus100.m handles the current internally (body-frame rotation,
%   relative velocity, Dnu_c term). We must NOT double-apply the current
%   by also computing a tau_current and adding it to tau — that would
%   apply current effects twice. Current goes ONLY via Vc/betaVc/w_c.
%
% CURRENT MODEL:
%   Constant horizontal current with optional Gauss-Markov slow variation.
%   Direction convention: betaVc is the NED direction FROM which current
%   flows (same as remus100.m convention). 0 = current flowing South
%   (from North), pi/2 = current flowing West (from East).
%
% WAVE DISTURBANCE MODEL:
%   Second-order filtered white noise (spectral factorisation approach,
%   Fossen 2021 Section 8.3). Approximates ITTC Modified Pierson-Moskowitz
%   spectrum via a 2nd-order transfer function:
%       H(s) = K_w * s / (s^2 + 2*zeta*omega_n*s + omega_n^2)
%   Significant wave height Hs and peak period Tp are configurable.
%   Only heave (Z), pitch (M) and roll (K) are driven — surge/sway/yaw
%   wave excitation is negligible for a submerged AUV.
%
% FUNCTIONS:
%   env_init          — build environment state struct from auv params
%   env_step          — one environment timestep, returns EnvBus values
%   current_step      — compute (Vc, betaVc, w_c) at current time
%   wave_step         — integrate wave filter, return tau_wave [6×1]
%   wave_filter_init  — compute 2nd-order filter params from Hs, Tp
%   gauss_markov_step — update a 1st-order Gauss-Markov process
%
% AUTHOR: AUV Simulation Project — Phase 7

% =========================================================================
function env_state = env_init(auv)
%% env_init  —  Initialise environment state from auv parameter struct
%
% Reads environment configuration from auv.env (added to auv_params.m).
% Call once before the simulation loop.

% Check that auv.env exists — if not, use safe zero defaults
if ~isfield(auv, 'env')
    warning('env_init: auv.env not found. Using zero environment.');
    env_state = env_zero();
    return
end

e = auv.env;

% --- Current state ---
env_state.Vc        = e.Vc_mean;       % mean current speed (m/s)
env_state.betaVc    = e.betaVc_mean;   % mean current direction NED (rad)
env_state.w_c       = e.w_c;           % vertical current (m/s)
env_state.Vc_gm     = e.Vc_mean;       % Gauss-Markov state for speed
env_state.beta_gm   = e.betaVc_mean;   % Gauss-Markov state for direction

% --- Wave filter state (2nd-order ODE, 3 channels: Z, K, M) ---
% State: [x1; x2] per channel where x1=position, x2=velocity
% Channels: 1=heave(Z), 2=roll(K), 3=pitch(M)
env_state.wave_x    = zeros(6, 1);     % [x1_Z; x2_Z; x1_K; x2_K; x1_M; x2_M]
env_state.wave_on   = e.wave_on;

% Compute wave filter parameters from Hs and Tp
if e.wave_on && e.Hs > 0
    [env_state.wf_Z] = wave_filter_init(e.Hs, e.Tp, auv.phys.rho, e.wave_scale_Z);
    [env_state.wf_K] = wave_filter_init(e.Hs * 0.3, e.Tp, auv.phys.rho, e.wave_scale_K);
    [env_state.wf_M] = wave_filter_init(e.Hs * 0.5, e.Tp, auv.phys.rho, e.wave_scale_M);
else
    env_state.wf_Z = wave_filter_init(0, 6, auv.phys.rho, 0);
    env_state.wf_K = wave_filter_init(0, 6, auv.phys.rho, 0);
    env_state.wf_M = wave_filter_init(0, 6, auv.phys.rho, 0);
end

env_state.dt        = auv.sim.Ts;
env_state.mu_Vc     = e.mu_Vc;         % Gauss-Markov decay rate for speed
env_state.mu_beta   = e.mu_betaVc;     % Gauss-Markov decay rate for direction
env_state.sigma_Vc  = e.sigma_Vc;      % noise std for speed variation
env_state.sigma_beta= e.sigma_betaVc;  % noise std for direction variation

end

% =========================================================================
function [Vc, betaVc, w_c, tau_env, env_state] = env_step(env_state, t) %#ok<INUSL>
%% env_step  —  One environment timestep
%
% INPUTS:
%   env_state   environment state struct from env_init
%   t           current simulation time (s) — available for future use
%
% OUTPUTS:
%   Vc          current speed (m/s) — passed to remus100.m
%   betaVc      current direction NED (rad) — passed to remus100.m
%   w_c         vertical current (m/s) — passed to remus100.m
%   tau_env     [6×1] wave + wind disturbance forces (N, N·m)
%   env_state   updated state

% Current
[Vc, betaVc, w_c, env_state] = current_step(env_state);

% Wave disturbance
[tau_wave, env_state] = wave_step(env_state);

% Wind (Phase 7: zero — implement in later extension)
tau_wind = zeros(6,1);

% Total environmental disturbance (NOT including current — that goes via Vc/betaVc)
tau_env = tau_wave + tau_wind;

end

% =========================================================================
function [Vc, betaVc, w_c, env_state] = current_step(env_state)
%% current_step  —  Update and return ocean current parameters
%
% Updates Gauss-Markov processes for slow current variation.
% Returns the current parameters for remus100.m.

dt = env_state.dt;

% Update Gauss-Markov speed variation
env_state.Vc_gm = gauss_markov_step(...
    env_state.Vc_gm, env_state.mu_Vc, env_state.sigma_Vc, dt);
Vc = max(0, env_state.Vc_gm);   % speed is non-negative

% Update Gauss-Markov direction variation
env_state.beta_gm = gauss_markov_step(...
    env_state.beta_gm, env_state.mu_beta, env_state.sigma_beta, dt);
betaVc = atan2(sin(env_state.beta_gm), cos(env_state.beta_gm));  % wrap

% Vertical current — constant in Phase 7
w_c = env_state.w_c;

% Store for next step
env_state.Vc    = Vc;
env_state.betaVc= betaVc;

end

% =========================================================================
function [tau_wave, env_state] = wave_step(env_state)
%% wave_step  —  Integrate 2nd-order wave filter, return disturbance forces
%
% Three independent 2nd-order filters, one per relevant DOF.
% State: [x1; x2] where x1 is force output, x2 is velocity state.
%
% Continuous model per channel:
%   ẋ1 = x2
%   ẋ2 = -omega_n^2 * x1 - 2*zeta*omega_n * x2 + Kw * w(t)
% Output: tau = x1
%
% Discretised with Euler forward (adequate at Ts=0.01s for wave periods > 2s).

tau_wave = zeros(6,1);

if ~env_state.wave_on
    return
end

dt = env_state.dt;

% Heave (Z — index 3)
[tau_wave(3), env_state.wave_x(1:2)] = ...
    wave_filter_step(env_state.wave_x(1:2), env_state.wf_Z, dt);

% Roll (K — index 4)
[tau_wave(4), env_state.wave_x(3:4)] = ...
    wave_filter_step(env_state.wave_x(3:4), env_state.wf_K, dt);

% Pitch (M — index 5)
[tau_wave(5), env_state.wave_x(5:6)] = ...
    wave_filter_step(env_state.wave_x(5:6), env_state.wf_M, dt);

end

% =========================================================================
function [tau_out, x_next] = wave_filter_step(x, wf, dt)
%% wave_filter_step  —  Euler integration of one 2nd-order wave channel
%
% x   = [x1; x2]  current filter state
% wf  = filter parameter struct: omega_n, zeta, Kw
% dt  = timestep

x1 = x(1);
x2 = x(2);
w  = randn;   % white noise input — reseeded each step

% Derivatives
dx1 = x2;
dx2 = -wf.omega_n^2 * x1 - 2*wf.zeta*wf.omega_n * x2 + wf.Kw * w;

% Euler forward
x_next = [x1 + dt*dx1;
          x2 + dt*dx2];

tau_out = x_next(1);   % filter output is x1

end

% =========================================================================
function wf = wave_filter_init(Hs, Tp, rho, scale) %#ok<INUSL>
%% wave_filter_init  —  Compute 2nd-order filter parameters from wave spec
%
% Spectral factorisation approach (Fossen 2021, Perez 2005):
%   The ITTC Modified PM spectrum has modal frequency omega_0 = 2*pi/Tp.
%   The 2nd-order filter H(s) = Kw*s / (s^2 + 2*zeta*omega_n*s + omega_n^2)
%   approximates the spectrum shape.
%
%   Parameters:
%     omega_n = omega_0  (filter natural frequency = modal wave frequency)
%     zeta    = 0.1      (lightly damped to produce peaked spectrum)
%     Kw      = sigma_wave * sqrt(2*zeta*omega_n)  (match variance)
%   where sigma_wave = Hs / 4  (significant height = 4 * std deviation)
%
% scale  — additional force amplitude scale factor (N per metre of Hs)

if Hs <= 0 || Tp <= 0
    wf.omega_n = 1.0;
    wf.zeta    = 0.1;
    wf.Kw      = 0;
    return
end

omega_0        = 2*pi / Tp;     % modal (peak) angular frequency (rad/s)
sigma_wave     = Hs / 4;        % wave elevation std deviation (m)

wf.omega_n     = omega_0;
wf.zeta        = 0.1;           % light damping — peaked spectrum
wf.Kw          = scale * sigma_wave * sqrt(2 * wf.zeta * wf.omega_n);

end

% =========================================================================
function x_next = gauss_markov_step(x, mu, sigma, dt)
%% gauss_markov_step  —  Euler step of 1st-order Gauss-Markov process
%
% dx/dt = -mu * x + sigma * w(t)   where w ~ N(0,1)
%
% mu    — decay rate (1/s): higher = faster mean reversion
%         mu = 0 → random walk (pure Brownian motion)
%         mu > 0 → mean-reverting (bounded variance)
% sigma — noise intensity (units per sqrt(s))

if sigma <= 0
    % Constant value — no variation
    x_next = x;
    return
end

w      = randn;
x_next = x + dt * (-mu * x + sigma * w);

end

% =========================================================================
function env_state = env_zero()
%% env_zero  —  Safe zero-environment state (fallback)

env_state.Vc        = 0;
env_state.betaVc    = 0;
env_state.w_c       = 0;
env_state.Vc_gm     = 0;
env_state.beta_gm   = 0;
env_state.wave_x    = zeros(6,1);
env_state.wave_on   = false;
env_state.wf_Z      = wave_filter_init(0, 6, 1026, 0);
env_state.wf_K      = wave_filter_init(0, 6, 1026, 0);
env_state.wf_M      = wave_filter_init(0, 6, 1026, 0);
env_state.dt        = 0.01;
env_state.mu_Vc     = 0;
env_state.mu_beta   = 0;
env_state.sigma_Vc  = 0;
env_state.sigma_beta= 0;

end
