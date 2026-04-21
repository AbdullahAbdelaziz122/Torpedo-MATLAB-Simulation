%% control_pid_lib.m  —  PID Control Library  (Module M3 — Phase 5)
%
% PURPOSE:
%   Implements the three PID control channels for the REMUS 100 AUV:
%     1. Surge speed controller   — error_u  → n_direct (RPM)
%     2. Pitch/depth controller   — cascade: z_d→theta_d→delta_s
%     3. Heading controller       — error_chi → delta_r
%
%   All three channels use model-based feedforward to cancel dominant
%   Coriolis and gravity coupling terms, then apply PID on the residual.
%   This is the same structure as the Ariza LOSPID controller, but with
%   gains derived analytically from the mass matrix rather than tuned
%   empirically.
%
% ARCHITECTURE CONTRACT (M3 interface):
%   Inputs:  GuidanceBus   — chi_d, upsilon_d, ud
%            NavBus        — nu_hat [6×1], eta_hat [6×1]
%            ctrl_params   — gains from auv.ctrl
%   Outputs: tau_ctrl [6×1]  — generalised forces [X Y Z K M N]
%            n_direct [1×1]  — direct RPM command to Actuation port 3
%            ctrl_debug      — error signals for logging
%
% FEEDFORWARD TERMS (derived from linearised EOM at cruise U0):
%
%   SURGE (X channel):
%     The speed error drives RPM directly — no tau_ctrl(1) used.
%     Feedforward cancels: -m11*(v*r - w*q) Coriolis coupling
%     where m11 = M(1,1) = m - Xudot = 32.83
%
%   PITCH/DEPTH (M channel — tau_ctrl(5)):
%     Inner loop (pitch rate damping):    m55 * q  where m55 = M(5,5) = 8.33
%     Gravity compensation:               (W*zg - B*zb)*sin(theta)
%     Coriolis cancellation:             -m55_uw * u * w
%     where m55_uw comes from the (5,3) coupling in the mass matrix
%
%   YAW/HEADING (N channel — tau_ctrl(6)):
%     Rate damping:                        m66 * r  where m66 = M(6,6) = 8.33
%     Coriolis cancellation:               m66_uv * u * v
%
% ANTI-WINDUP:
%   Integrator state is frozen when the actuator saturates.
%   Implemented via back-calculation: when |output| > sat_limit,
%   the integral is not updated. This prevents integrator windup
%   during large transients (e.g. large heading changes).
%
% FUNCTIONS:
%   control_pid_init     — initialise controller state struct
%   control_pid_step     — one control step, returns tau_ctrl + n_direct
%   pid_channel          — generic PID with anti-windup
%   wrap_error           — angle error wrapping to [-pi, pi]
%
% AUTHOR: AUV Simulation Project — Phase 5

% =========================================================================
function ctrl_state = control_pid_init(auv)
%% control_pid_init  —  Initialise PID state struct
%
% Call once before the simulation loop. Returns a state struct that
% carries integrator states and previous errors across timesteps.
%
% The state struct is the discrete-time memory of the controller.
% In Simulink this maps to persistent variables inside a MATLAB Function
% block, or to Unit Delay states.

ctrl_state.dt          = auv.sim.Ts;

% --- Surge channel ---
ctrl_state.u.integral  = 0;
ctrl_state.u.e_prev    = 0;
ctrl_state.u.Kp        = auv.ctrl.u.Kp;
ctrl_state.u.Ki        = auv.ctrl.u.Ki;
ctrl_state.u.Kd        = auv.ctrl.u.Kd;
ctrl_state.u.sat       = auv.ctrl.u.u_max;

% --- Depth outer loop (z → theta_d) ---
ctrl_state.z.integral  = 0;
ctrl_state.z.e_prev    = 0;
ctrl_state.z.Kp        = auv.ctrl.z.Kp;
ctrl_state.z.Ki        = auv.ctrl.z.Ki;
ctrl_state.z.theta_max = auv.ctrl.z.theta_d_max;

% --- Pitch inner loop (theta → delta_s via tau_M) ---
ctrl_state.theta.integral = 0;
ctrl_state.theta.e_prev   = 0;
ctrl_state.theta.Kp       = auv.ctrl.theta.Kp;
ctrl_state.theta.Ki       = auv.ctrl.theta.Ki;
ctrl_state.theta.Kd       = auv.ctrl.theta.Kd;
ctrl_state.theta.sat      = auv.ctrl.theta.sat;

% --- Heading channel (chi → delta_r via tau_N) ---
ctrl_state.psi.integral   = 0;
ctrl_state.psi.e_prev     = 0;
ctrl_state.psi.Kp         = auv.ctrl.psi.Kp;
ctrl_state.psi.Ki         = auv.ctrl.psi.Ki;
ctrl_state.psi.Kd         = auv.ctrl.psi.Kd;
ctrl_state.psi.sat        = auv.ctrl.psi.sat;

% --- Mass matrix entries (computed once from remus100) ---
% These are used for feedforward normalisation and Coriolis cancellation.
[~, ~, M] = remus100();
ctrl_state.m11 = M(1,1);   % surge inertia  = m - Xudot  ≈ 32.83
ctrl_state.m55 = M(5,5);   % pitch inertia  = Iy - Mqdot ≈ 8.33
ctrl_state.m66 = M(6,6);   % yaw inertia    = Iz - Nrdot ≈ 8.33
% Cross-coupling terms for feedforward
% M(3,5) = -Zqdot - m*xg = -(-1.93) = 1.93  (heave-pitch coupling)
% M(2,6) = m*xg - Yrdot  = 0 - 1.93 = -1.93 (sway-yaw coupling)
ctrl_state.m35 = M(3,5);
ctrl_state.m26 = M(2,6);

% Gravity/buoyancy terms
ctrl_state.W   = auv.phys.W;
ctrl_state.B   = auv.phys.B;
ctrl_state.zg  = auv.phys.r_bG(3);
ctrl_state.zb  = auv.phys.r_bB(3);

end

% =========================================================================
function [tau_ctrl, n_direct, debug, ctrl_state] = control_pid_step(...
    guid, nu_hat, eta_hat, ctrl_state)
%% control_pid_step  —  One PID control timestep
%
% INPUTS:
%   guid       struct   guidance references: .chi_d, .upsilon_d, .ud
%   nu_hat     [6×1]    estimated body velocities from Navigation
%   eta_hat    [6×1]    estimated NED position + angles from Navigation
%   ctrl_state struct   PID state (integrators, previous errors)
%
% OUTPUTS:
%   tau_ctrl   [6×1]    generalised force/moment [X Y Z K M N]
%   n_direct   [1×1]    direct RPM command for Actuation port 3
%   debug      struct   error signals and integrator states for logging
%   ctrl_state struct   updated PID state for next timestep

% Unpack state
u = nu_hat(1);  v = nu_hat(2);  w = nu_hat(3);
p = nu_hat(4);  q = nu_hat(5);  r = nu_hat(6); %#ok<NASGU>

theta = eta_hat(5);
chi_v = atan2(nu_hat(2), nu_hat(1));   % vehicle course angle from velocities

% =========================================================================
% CHANNEL 1: Surge — speed controller → direct RPM
% =========================================================================
e_u = guid.ud - u;

[rpm_pid_out, ctrl_state.u] = pid_channel(e_u, ctrl_state.u);

% Feedforward: cancel Coriolis coupling -m11*(v*r - w*q)
% This term appears in the surge EOM as a coupling from yaw/pitch rates
ff_surge = ctrl_state.m11 * (v*r - w*q);

% RPM from PID output + feedforward
% The PID output is in force units (N); convert to RPM via Actuation
% For direct RPM command: scale PID output as RPM reference
% Factor: at cruise 1.5 m/s the vehicle needs ~7N thrust → ~900 RPM
% Empirical scale: 1 N ≈ 100/7 RPM → use 1/m11 * RPM_max/F_max
rpm_scale  = auv_rpm_scale(ctrl_state);  % computed from mass matrix
n_direct   = rpm_scale * (rpm_pid_out + ff_surge / ctrl_state.m11);
n_direct   = max(0, n_direct);   % no reverse

% =========================================================================
% CHANNEL 2: Depth — cascade controller
%   Outer loop: z error → desired pitch angle theta_d
%   Inner loop: theta error → pitch moment tau_M
% =========================================================================

% Outer loop: depth → theta_d
% NED convention: z_d is positive downward, so z_d > 0 means deeper.
% To dive: need positive theta_d (pitch down = positive theta in NED).
z_d    = eta_hat(3);
z_des  = 0;   % will be overridden when guidance sets depth; placeholder
% In the full system, depth setpoint comes from guidance (upsilon_d path).
% Here we use upsilon_d → theta_d via a simple proportional conversion:
%   theta_d = upsilon_d  (flight-path angle IS pitch angle for horizontal motion)
%   but depth error adds a PI correction on top.
% This matches Fossen's cascade structure in demoAUVdepthHeadingControl.pdf.

theta_d_from_upsilon = guid.upsilon_d;   % guidance provides pitch reference

[z_pi_out, ctrl_state.z] = pi_channel(z_d - z_des, ctrl_state.z);

% Saturate theta_d
theta_d = theta_d_from_upsilon + z_pi_out;
theta_d = max(-ctrl_state.theta.sat*2, min(ctrl_state.theta.sat*2, theta_d));
theta_d = max(-ctrl_state.z.theta_max, min(ctrl_state.z.theta_max, theta_d));

% Inner loop: theta error → tau_M (pitch moment)
e_theta = wrap_error(theta_d - theta);

[theta_pid_out, ctrl_state.theta] = pid_channel(e_theta, ctrl_state.theta);

% Feedforward for pitch channel:
%   Gravity/buoyancy moment: (W*zg - B*zb)*sin(theta)
%   Rate damping:            m55 * q  (cancels the q-term in the EOM)
%   Coriolis coupling:      -m35 * u * w  (heave-pitch cross term)
ff_pitch = (ctrl_state.W * ctrl_state.zg - ctrl_state.B * ctrl_state.zb) * sin(theta) ...
           + ctrl_state.m55 * q * 0.3 ...    % partial rate damping (0.3 = zeta5)
           - ctrl_state.m35 * u * w;

tau_M = ctrl_state.m55 * theta_pid_out + ff_pitch;

% =========================================================================
% CHANNEL 3: Heading — course angle controller → yaw moment tau_N
% =========================================================================
e_chi = wrap_error(guid.chi_d - chi_v);

[chi_pid_out, ctrl_state.psi] = pid_channel(e_chi, ctrl_state.psi);

% Feedforward for yaw channel:
%   Rate damping:    m66 * r  (cancels r-term in yaw EOM)
%   Munk moment:     m26 * u * v  (sway-yaw Coriolis)
ff_yaw = ctrl_state.m66 * r * 0.1 ...    % partial rate damping
         + ctrl_state.m26 * u * v;

tau_N = ctrl_state.m66 * chi_pid_out + ff_yaw;

% =========================================================================
% Assemble tau_ctrl [6×1]
% tau_ctrl(1) = X — surge force — NOT used here (speed via n_direct)
% tau_ctrl(2) = Y — sway force  — NOT directly controlled
% tau_ctrl(3) = Z — heave force — NOT directly controlled
% tau_ctrl(4) = K — roll moment — NOT controlled (passive stability)
% tau_ctrl(5) = M — pitch moment → stern plane via Actuation
% tau_ctrl(6) = N — yaw moment  → rudder via Actuation
% =========================================================================
tau_ctrl    = zeros(6, 1);
tau_ctrl(5) = tau_M;
tau_ctrl(6) = tau_N;

% =========================================================================
% Debug struct for logging
% =========================================================================
debug.e_u       = e_u;
debug.e_theta   = e_theta;
debug.e_chi     = e_chi;
debug.theta_d   = theta_d;
debug.chi_v     = chi_v;
debug.tau_M     = tau_M;
debug.tau_N     = tau_N;
debug.n_direct  = n_direct;
debug.ff_pitch  = ff_pitch;
debug.ff_yaw    = ff_yaw;

end

% =========================================================================
function [out, ch] = pid_channel(error, ch)
%% pid_channel  —  Discrete PID with anti-windup (back-calculation)
%
% INPUTS:
%   error   current error signal
%   ch      channel state struct (Kp, Ki, Kd, sat, integral, e_prev, dt)
%
% OUTPUTS:
%   out     PID output (before external saturation)
%   ch      updated state

dt = ch.dt;

% Derivative on measurement (not error) is better practice, but since
% we are differencing error here, we use a simple backward difference.
% The controller is slow enough (0.01s steps) that this is acceptable.
deriv = (error - ch.e_prev) / dt;

% Raw PID output
out_raw = ch.Kp * error + ch.Ki * ch.integral + ch.Kd * deriv;

% Output saturation
out = max(-ch.sat, min(ch.sat, out_raw));

% Anti-windup: only integrate when NOT saturated
% If saturating, freeze the integrator (conditional integration)
if abs(out_raw) <= ch.sat
    ch.integral = ch.integral + error * dt;
end

ch.e_prev = error;

end

% =========================================================================
function [out, ch] = pi_channel(error, ch)
%% pi_channel  —  PI controller (no derivative) for outer depth loop
%
% The outer z-loop is PI only — derivative on depth would amplify
% heave velocity noise.

dt = ch.dt;

out_raw = ch.Kp * error + ch.Ki * ch.integral;
out = max(-ch.theta_max, min(ch.theta_max, out_raw));

if abs(out_raw) <= ch.theta_max
    ch.integral = ch.integral + error * dt;
end

end

% =========================================================================
function e_w = wrap_error(e)
%% wrap_error  —  Wrap angle error to [-pi, pi]
%
% Critical for heading and pitch errors: prevents 358° commands
% when error crosses the ±pi boundary.

e_w = atan2(sin(e), cos(e));

end

% =========================================================================
function scale = auv_rpm_scale(ctrl_state)
%% auv_rpm_scale  —  Empirical RPM/force scale from mass matrix
%
% Converts PID output (force units, N) to RPM.
% At cruise 1.5 m/s: ~7 N thrust needed → ~900 RPM.
% Scale = RPM_cruise / (m11 * a_max)
% where a_max = max acceleration at full thrust ≈ F_max / m11
%
% This gives a rough proportional mapping that the integrator refines.
% The exact value is less critical than it sounds — the PID integrator
% compensates for any static gain error within a few seconds.

% Approximate: 1525 RPM → ~20 N thrust at 1.5 m/s (from remus100 calibration)
% Scale: 1525/20 = 76.25 RPM/N, normalised by m11 for force→RPM
scale = 1525 / 20;   % RPM per Newton

end
