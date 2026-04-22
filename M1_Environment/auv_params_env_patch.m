%% auv_params_env_patch.m  —  Environment parameters for auv struct
%
% PURPOSE:
%   Adds the auv.env sub-struct to an already-loaded auv struct.
%   Run this AFTER auv_params.m if you want environment enabled.
%
%   Alternatively, paste the block below directly into auv_params.m
%   at the end of section 10 (after the nav section).
%
% USAGE:
%   >> buses; auv_params; auv_params_env_patch
%   OR paste the auv.env block into auv_params.m permanently.
%
% CURRENT MODEL:
%   Constant mean current with optional slow Gauss-Markov variation.
%   betaVc convention (matches remus100.m):
%     0       = current flowing South  (FROM the North)
%     pi/2    = current flowing West   (FROM the East)
%     pi      = current flowing North  (FROM the South)
%
% WAVE MODEL:
%   ITTC Modified PM spectrum via 2nd-order filtered white noise.
%   Hs = 0 disables waves entirely.
%   Affects: heave (Z), roll (K), pitch (M) only.
%   Wave forces on a submerged AUV attenuate with depth — at the REMUS
%   operating depth (2-20m), wave effects are real but moderate.

if ~exist('auv','var')
    error('Run auv_params.m first, then re-run this script.');
end

% =========================================================================
% Ocean current
% =========================================================================
auv.env.Vc_mean     = 0.3;      % mean current speed (m/s)
                                 % 0 = no current, 0.3 = moderate, 0.5 = strong
auv.env.betaVc_mean = pi/4;     % mean current direction NED (rad)
                                 % pi/4 = from NW (current flowing SE)
auv.env.w_c         = 0.0;      % vertical current speed (m/s)

% Gauss-Markov slow variation around mean
% mu = 0  → random walk (unbounded, not recommended)
% mu > 0  → mean-reverting around Vc_mean / betaVc_mean
auv.env.mu_Vc       = 0.01;     % speed decay rate (1/s) — slow variation
auv.env.mu_betaVc   = 0.005;    % direction decay rate (1/s)
auv.env.sigma_Vc    = 0.02;     % speed noise intensity (m/s / sqrt(s))
auv.env.sigma_betaVc= 0.01;     % direction noise intensity (rad / sqrt(s))

% =========================================================================
% Wave disturbance
% =========================================================================
auv.env.wave_on     = true;     % false = no waves (calm water)
auv.env.Hs          = 0.5;      % significant wave height (m)
                                 % 0.5m = mild, 1.5m = moderate, 3m = rough
auv.env.Tp          = 6.0;      % peak wave period (s)
                                 % Typical ocean: 6-12s

% Force amplitude scale factors (N per unit wave filter output)
% These translate the dimensionless wave filter output to physical forces.
% Calibrated so that Hs=1m at 2m depth produces approximately:
%   heave:  ~15 N peak   pitch:  ~8 N·m peak   roll: ~5 N·m peak
auv.env.wave_scale_Z = 15.0;   % heave force scale (N/m)
auv.env.wave_scale_K =  5.0;   % roll  moment scale (N·m/m)
auv.env.wave_scale_M =  8.0;   % pitch moment scale (N·m/m)

% =========================================================================
% Quick-select presets (uncomment one to apply)
% =========================================================================
% --- Calm water (testing, Phase 1-6 equivalent) ---
% auv.env.Vc_mean=0; auv.env.sigma_Vc=0; auv.env.wave_on=false;

% --- Mild current, no waves ---
% auv.env.Vc_mean=0.2; auv.env.sigma_Vc=0; auv.env.wave_on=false;

% --- Moderate sea state ---
% auv.env.Vc_mean=0.3; auv.env.Hs=1.0; auv.env.Tp=7.0;

% --- Rough conditions ---
% auv.env.Vc_mean=0.5; auv.env.Hs=2.0; auv.env.Tp=9.0;

assignin('base', 'auv', auv);
fprintf('auv.env parameters loaded.\n');
fprintf('  Current: Vc=%.2f m/s, betaVc=%.1f deg\n', ...
    auv.env.Vc_mean, rad2deg(auv.env.betaVc_mean));
fprintf('  Waves:   Hs=%.1fm, Tp=%.1fs, wave_on=%d\n', ...
    auv.env.Hs, auv.env.Tp, auv.env.wave_on);
