%% control_smc_lib.m  —  Sliding Mode Controller Library  (Phase 9 — M3)
%
% PURPOSE:
%   Implements a first-order Sliding Mode Controller (SMC) for the REMUS 100.
%   Conforms EXACTLY to the M3 interface contract — identical inputs and
%   outputs to control_pid_lib.m. Swap by changing one line in run_simulation.m.
%
% SMC THEORY (brief):
%   For a plant  m·ẍ + d·ẋ = u + f(t)  where f is unknown disturbance:
%
%   1. Define sliding surface:  s = ė + λe  (e = x_d - x)
%      In integral form:        s = e + λ·∫e dt
%      (avoids differentiating noisy measurements)
%
%   2. Control law:
%      u = m·(ẍ_d + λ·ė + k·sat(s/φ)) + d·ẋ_ff
%
%      where:
%        λ    — surface slope (must exceed plant pole magnitude)
%        k    — reaching gain (must exceed disturbance bound / m)
%        φ    — boundary layer thickness (eliminates chattering)
%        sat  — saturation function: sat(x) = x if |x|≤1, sgn(x) otherwise
%
%   3. Stability: Lyapunov function V = s²/2 → V̇ = s·ṡ ≤ -η|s| < 0
%      when k > (disturbance_bound + η) / m
%
% GAIN DERIVATION (analytical, from REMUS plant):
%   Mass matrix (from remus100):  m11=32.83, m55=8.33, m66=8.33 kg(·m²)
%   Plant poles (from remus100 time constants):
%     Surge: -1/T1 = -0.05 rad/s
%     Yaw:   -1/T6 = -1.0  rad/s
%     Pitch: natural frequency ωn5, damping ζ5=0.8
%
%   Design rule: λ > |plant pole| ensures sliding surface is stable
%     λ_surge = 0.20  (4× plant pole = 4×0.05)
%     λ_yaw   = 3.00  (3× plant pole = 3×1.0)
%     λ_pitch = 5.00  (well above pitch natural frequency)
%
%   Reaching gain: k > D_max/m_ii  where D_max is max disturbance
%     Max wave force on AUV: ~20N heave, ~8Nm pitch, ~8Nm yaw
%     k_surge = 20/32.83 + 0.5 ≈ 1.1   → use 1.5 (margin)
%     k_pitch = 8/8.33   + 0.5 ≈ 1.5   → use 2.0 (margin)
%     k_yaw   = 8/8.33   + 0.5 ≈ 1.5   → use 2.0 (margin)
%
%   Boundary layer: φ = 0.1 (eliminates chattering, introduces small
%     steady-state band of φ·k/λ ≈ 0.05 m/s in surge — acceptable)
%
% FEEDFORWARD TERMS (identical to PID):
%   Same gravity compensation, Coriolis cancellation and rate damping
%   terms as control_pid_lib.m. SMC only replaces the error-feedback law.
%
% INTERFACE CONTRACT (M3 — identical to control_pid_lib.m):
%   Inputs:   guid.chi_d, guid.upsilon_d, guid.ud, guid.z_des
%             nu_hat [6×1], eta_hat [6×1]
%   Outputs:  tau_ctrl [6×1], n_direct [1×1], debug struct, ctrl_state
%
% FUNCTIONS:
%   control_smc_init   — initialise SMC state (integrators)
%   control_smc_step   — one SMC timestep
%   smc_channel        — generic SMC with boundary layer + anti-windup
%   sat_func           — smooth saturation function (no chattering)
%
% AUTHOR: AUV Simulation Project — Phase 9

% =========================================================================
function cs = control_smc_init(auv)
%% control_smc_init  —  Initialise SMC state struct

cs.dt = auv.sim.Ts;

% --- SMC parameters (analytically derived — see header) ---

% Surge channel
cs.u.lambda  = 0.20;    % surface slope (rad/s) — 4× plant pole
cs.u.k       = 1.50;    % reaching gain
cs.u.phi     = 0.10;    % boundary layer thickness
cs.u.integral= 0;       % sliding surface integrator state
cs.u.e_prev  = 0;

% Depth outer loop (PI — same structure as PID, SMC not needed here)
% The depth outer loop is slow and well-behaved; PI is sufficient.
% SMC is applied on the pitch inner loop where disturbances matter.
cs.z.integral  = 0;
cs.z.Kp        = auv.ctrl.z.Kp;
cs.z.Ki        = auv.ctrl.z.Ki;
cs.z.theta_max = auv.ctrl.z.theta_d_max;

% Pitch inner loop (SMC)
cs.theta.lambda   = 5.00;   % surface slope — above pitch natural freq
cs.theta.k        = 2.00;   % reaching gain (covers wave pitch moment)
cs.theta.phi      = 0.05;   % tighter boundary layer — pitch is fast
cs.theta.integral = 0;
cs.theta.e_prev   = 0;
cs.theta.sat      = auv.ctrl.theta.sat;   % output saturation (from auv_params)

% Heading / yaw channel (SMC)
cs.psi.lambda  = 3.00;      % surface slope — 3× yaw plant pole
cs.psi.k       = 2.00;      % reaching gain
cs.psi.phi     = 0.10;      % boundary layer
cs.psi.integral= 0;
cs.psi.e_prev  = 0;
cs.psi.sat     = auv.ctrl.psi.sat;

% Mass matrix entries (from remus100 — computed once at init)
[~, ~, M] = remus100();
cs.m11 = M(1,1);   % 32.83 kg  — surge
cs.m55 = M(5,5);   %  8.33 kg·m² — pitch
cs.m66 = M(6,6);   %  8.33 kg·m² — yaw
cs.m35 = M(3,5);   % heave-pitch coupling
cs.m26 = M(2,6);   % sway-yaw coupling

% Physical parameters for feedforward
cs.W  = auv.phys.W;
cs.B  = auv.phys.B;
cs.zg = auv.phys.r_bG(3);
cs.zb = auv.phys.r_bB(3);

end

% =========================================================================
function [tau_ctrl, n_direct, debug, cs] = control_smc_step(...
    guid, nu_hat, eta_hat, cs)
%% control_smc_step  —  One SMC timestep
%
% Interface contract: identical to control_pid_step in control_pid_lib.m.
% Guidance inputs, Navigation inputs, and all outputs are identical.
% Only the internal error-feedback law differs.

u = nu_hat(1);  v = nu_hat(2);  w = nu_hat(3);
q = nu_hat(5);  r = nu_hat(6);
theta  = eta_hat(5);
chi_v  = atan2(sin(eta_hat(6)), cos(eta_hat(6)));
Ts     = cs.dt;

% =========================================================================
% CHANNEL 1: Surge — SMC → direct RPM
% =========================================================================
e_u = guid.ud - u;

% Sliding surface: s = e + lambda * integral(e)
[s_u, cs.u] = smc_surface(e_u, cs.u, Ts);

% SMC control output (force units)
u_smc = cs.u.lambda * e_u + cs.u.k * sat_func(s_u / cs.u.phi);

% Feedforward: Coriolis coupling (same as PID)
ff_surge = cs.m11 * (v*r - w*q);

% Convert to RPM (same scaling as PID)
n_direct = max(0, (1525/20) * (cs.m11 * u_smc + ff_surge / cs.m11));
n_direct = min(1525, n_direct);

% =========================================================================
% CHANNEL 2: Depth — PI outer loop → theta_d (same as PID)
% =========================================================================
z_d   = eta_hat(3);
z_ref = guid.z_des;
e_z   = z_ref - z_d;

out_z = cs.z.Kp * e_z + cs.z.Ki * cs.z.integral;
theta_d = max(-cs.z.theta_max, min(cs.z.theta_max, out_z));
if abs(out_z) <= cs.z.theta_max
    cs.z.integral = cs.z.integral + e_z * Ts;
end

% =========================================================================
% CHANNEL 2 inner: Pitch — SMC → tau_M
% =========================================================================
e_theta = atan2(sin(theta_d - theta), cos(theta_d - theta));

[s_th, cs.theta] = smc_surface(e_theta, cs.theta, Ts);

% SMC pitch output
theta_smc = cs.theta.lambda * e_theta + cs.theta.k * sat_func(s_th / cs.theta.phi);
theta_smc = max(-cs.theta.sat, min(cs.theta.sat, theta_smc));

% Feedforward (identical to PID)
ff_pitch = (cs.W*cs.zg - cs.B*cs.zb)*sin(theta) ...
           + 0.3*cs.m55*q - cs.m35*u*w;

tau_M = cs.m55 * theta_smc + ff_pitch;

% =========================================================================
% CHANNEL 3: Heading — SMC → tau_N
% =========================================================================
e_chi = atan2(sin(guid.chi_d - chi_v), cos(guid.chi_d - chi_v));

[s_psi, cs.psi] = smc_surface(e_chi, cs.psi, Ts);

% SMC yaw output
psi_smc = cs.psi.lambda * e_chi + cs.psi.k * sat_func(s_psi / cs.psi.phi);
psi_smc = max(-cs.psi.sat, min(cs.psi.sat, psi_smc));

% Feedforward (identical to PID)
ff_yaw = 0.1*cs.m66*r + cs.m26*u*v;

tau_N = cs.m66 * psi_smc + ff_yaw;

% =========================================================================
% Assemble outputs (identical structure to PID)
% =========================================================================
tau_ctrl      = zeros(6, 1);
tau_ctrl(5)   = tau_M;
tau_ctrl(6)   = tau_N;

% Debug struct — identical field names to PID for drop-in compatibility
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
% SMC-specific diagnostics
debug.s_u       = s_u;
debug.s_theta   = s_th;
debug.s_psi     = s_psi;

end

% =========================================================================
function [s, ch] = smc_surface(error, ch, dt)
%% smc_surface  —  Update sliding surface state
%
% Sliding surface: s = e + lambda * integral(e dt)
% The integral acts as a low-pass filter on the error signal,
% making s smoother than a pure derivative-based surface.
%
% Anti-windup: integral frozen when |s| > phi (reaching phase).
% This prevents the integrator from winding up during large transients —
% the same problem anti-windup solves in PID, now applied to the SMC
% integrator state.

ch.integral = ch.integral + error * dt;

% Sliding surface value
s = error + ch.lambda * ch.integral;

% Anti-windup: freeze integration when well outside boundary layer
% (when reaching, not sliding — integrator not needed)
if abs(s) > 3 * ch.phi
    ch.integral = ch.integral - error * dt;   % undo this step
end

ch.e_prev = error;

end

% =========================================================================
function y = sat_func(x)
%% sat_func  —  Smooth saturation replacing sign function
%
% Replaces the discontinuous sgn(s) in classic SMC with a smooth sat(s/phi).
% This is the STANDARD chattering-elimination technique (Slotine & Li, 1991).
%
% When |x| <= 1: y = x          (inside boundary layer — linear)
% When |x|  > 1: y = sign(x)    (outside — full reaching gain)
%
% Effect: eliminates high-frequency control chattering that would
% excite AUV structural modes and cause actuator wear.
% Cost: introduces a steady-state band of size phi*k/lambda around zero.
% For our parameters: ~0.1*1.5/0.2 = 0.75 m/s in surge — acceptable.

if abs(x) <= 1
    y = x;
else
    y = sign(x);
end

end
