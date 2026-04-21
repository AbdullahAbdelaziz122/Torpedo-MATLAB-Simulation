%% auv_params.m  —  AUV Simulation Parameters
%
% PURPOSE:
%   Defines the 'auv' parameter struct used throughout the simulation.
%   Every module reads from this struct — no magic numbers in subsystems.
%   Run AFTER buses.m, BEFORE opening any Simulink model.
%
% USAGE:
%   >> buses          % must run first
%   >> auv_params     % then this
%
% OUTPUT:
%   auv             — parameter struct in base workspace
%
% SECTIONS:
%   1. Simulation configuration
%   2. Vehicle physical properties
%   3. Added mass coefficients
%   4. Damping coefficients
%   5. Actuator geometry and limits
%   6. Propulsion model
%   7. Initial conditions
%   8. Control gains (Phase 5 — placeholders here)
%   9. Guidance parameters (Phase 6 — placeholders here)
%   10. Navigation / sensor noise (Phase 4)
%
% Source: Fossen (2021) remus100.m, Allen et al. (2000), Prestero (2001)

% =========================================================================
% 1.  Simulation configuration
% =========================================================================
auv.sim.Ts          = 0.01;     % Fundamental sample time (s) — 100 Hz
auv.sim.Ts_HIL      = 0.02;     % HIL communication rate (s) — 50 Hz
auv.sim.T_end       = 300;      % Default simulation duration (s)
auv.sim.solver      = 'ode4';   % Fixed-step RK4 for real-time
% For desktop development use 'ode45' — switch to 'ode4' for RT

% =========================================================================
% 2.  Vehicle physical properties
% =========================================================================
auv.phys.L          = 1.6;      % Vehicle length (m)
auv.phys.D          = 0.19;     % Cylinder diameter (m)
auv.phys.m          = 31.9;     % Mass (kg)
auv.phys.g          = 9.8100;   % Gravity at 63.4° lat — Trondheim (m/s²)
auv.phys.rho        = 1026;     % Seawater density (kg/m³)
auv.phys.W          = auv.phys.m * auv.phys.g;  % Weight (N)
auv.phys.B          = auv.phys.W;               % Buoyancy (N) — neutral
% NOTE: For realistic passive surfacing set B = W * 1.001
% The remus100.m function overrides this internally — these are
% used in your own modules, not passed to remus100.m directly.

% Center of gravity w.r.t. body origin [x, y, z] (m)
auv.phys.r_bG       = [0; 0; 0.02];   % CG is 2 cm below geometric center

% Center of buoyancy w.r.t. body origin
auv.phys.r_bB       = [0; 0; 0];      % CB at geometric center

% Moments of inertia (kg·m²)
auv.phys.Ix         = 0.177;
auv.phys.Iy         = 3.45;
auv.phys.Iz         = 3.45;   % Iz = Iy for axisymmetric body

% =========================================================================
% 3.  Added mass coefficients (Imlay 1961, prolate spheroid)
%     Sign convention: MA is defined as negative definite.
%     M = MRB + MA  where MA entries are negative.
% =========================================================================
auv.added.Xudot     = -0.93;   % Surge added mass (kg)
auv.added.Yvdot     = -35.5;   % Sway added mass (kg)   — ~1× physical mass
auv.added.Zwdot     = -35.5;   % Heave added mass (kg)  — equals sway (axisym)
auv.added.Kpdot     = -0.0;    % Roll added MOI (kg·m²)
auv.added.Mqdot     = -4.88;   % Pitch added MOI (kg·m²)
auv.added.Nrdot     = -4.88;   % Yaw added MOI (kg·m²)  — equals pitch

% Cross-coupling added mass terms (Prestero notation)
auv.added.Yrdot     =  1.93;
auv.added.Nvdot     =  1.93;
auv.added.Zqdot     = -1.93;
auv.added.Mwdot     = -1.93;

% =========================================================================
% 4.  Damping coefficients
% =========================================================================
% Linear (time-constant based, Fossen Dmtrx formulation)
auv.damp.T1         = 20;      % Surge time constant (s)
auv.damp.T2         = 20;      % Sway time constant (s)
auv.damp.T6         = 1;       % Yaw time constant (s)
auv.damp.zeta4      = 0.3;     % Roll relative damping ratio
auv.damp.zeta5      = 0.8;     % Pitch relative damping ratio

% Nonlinear quadratic drag coefficients (body-frame, from Prestero 2001)
auv.damp.Xuu        = -1.62;   % Surge quadratic drag (N·s²/m²)
auv.damp.Yvv        = -1310;   % Sway cross-flow drag
auv.damp.Yrr        =  0.632;
auv.damp.Zww        = -131;    % Heave cross-flow drag
auv.damp.Zqq        = -0.632;
auv.damp.Kpp        = -0.13;   % Roll resistance
auv.damp.Mqq        = -188;    % Pitch cross-flow
auv.damp.Nrr        = -94.0;   % Yaw cross-flow

% =========================================================================
% 5.  Actuator geometry and limits
% =========================================================================
% Fin geometry
auv.act.S_fin       = 0.00665; % Individual fin area (m²)
auv.act.delta_max   = deg2rad(20);  % Maximum fin deflection (rad) = 0.349 rad
auv.act.delta_min   = -auv.act.delta_max;

% Rudder (tail, horizontal plane)
auv.act.CL_delta_r  = 0.5;     % Rudder lift coefficient
auv.act.A_r         = 2 * auv.act.S_fin;  % Rudder area (m²)
auv.act.x_r         = -auv.phys.L/2;     % Rudder x-position (m)

% Stern planes (vertical plane)
auv.act.CL_delta_s  = 0.7;     % Stern-plane lift coefficient
auv.act.A_s         = 2 * auv.act.S_fin;  % Stern-plane area (m²)
auv.act.x_s         = -auv.phys.L/2;     % Stern-plane x-position (m)

% Propeller limits
auv.act.n_max       = 1525;    % Max RPM
auv.act.n_min       = 0;       % Min RPM (no reverse for AUV)

% PWM mapping  [actuator_min, actuator_max] → [PWM_min, PWM_max] µs
% Standard RC servo/ESC: 1000 µs = min, 1500 µs = neutral, 2000 µs = max
auv.act.pwm_fin_neutral = 1500;    % Neutral PWM for fins (µs)
auv.act.pwm_fin_range   = 500;     % ±500 µs for ±20° deflection
auv.act.pwm_esc_min     = 1000;    % ESC armed/stopped (µs)
auv.act.pwm_esc_max     = 2000;    % ESC full throttle (µs)

% =========================================================================
% 6.  Propulsion model (Allen et al. 2000, Wageningen B-series)
% =========================================================================
auv.prop.D_prop     = 0.14;    % Propeller diameter (m)
auv.prop.t_prop     = 0.1;     % Thrust deduction number
auv.prop.wake_frac  = 0.944;   % Wake fraction: Va = wake_frac * U
auv.prop.KT_0       = 0.4566;  % Thrust coefficient at Ja=0
auv.prop.KQ_0       = 0.0700;  % Torque coefficient at Ja=0
auv.prop.KT_max     = 0.1798;  % Thrust coefficient at Ja_max
auv.prop.KQ_max     = 0.0312;  % Torque coefficient at Ja_max
auv.prop.Ja_max     = 0.6632;  % Advance ratio at max speed

% =========================================================================
% 7.  Initial conditions
% =========================================================================
% State vector layout: x = [u v w p q r x_n y_e z_d phi theta psi]'
%                           1 2 3 4 5 6  7   8   9   10   11   12
auv.ic.nu           = [0; 0; 0; 0; 0; 0];   % Zero initial velocity
auv.ic.eta          = [0; 0; 0; 0; 0; 0];   % Start at NED origin, level
auv.ic.x0           = [auv.ic.nu; auv.ic.eta];  % Full 12-state IC vector

% Common test initial conditions
auv.ic.x0_surge_test = [1.5; 0; 0; 0; 0; 0;    % u=1.5 m/s, rest zero
                          0;  0; 0; 0; 0; 0];
auv.ic.x0_helix = [0.5; 0; 0; 0; 0; 3*pi/4;    % Near helix start
                   60; 3; 1; 0; 0; 3*pi/4];

% =========================================================================
% 8.  Control gains  (Phase 5 — tuned values will replace these)
%     Notation: ctrl.<channel>.<gain>
%     Channels: surge (u), depth (z), pitch (theta), heading (psi), yaw (r)
% =========================================================================
% Surge PID
auv.ctrl.u.Kp       = 25.0;
auv.ctrl.u.Ki       = 0.8;     % Pure PD for surge
auv.ctrl.u.Kd       = 0.5;
auv.ctrl.u.u_max    = 1525;      % RPM equivalent upper limit
auv.ctrl.u.u_min    = 0;

% Depth / pitch cascade — outer z-loop
auv.ctrl.z.Kp       = 1.2;    % z error → theta_d
auv.ctrl.z.Ki       = 0.15;
auv.ctrl.z.theta_d_max = deg2rad(20);   % Max commanded pitch (rad)

% Depth / pitch cascade — inner theta-loop
auv.ctrl.theta.Kp   = 10.0;
auv.ctrl.theta.Ki   = 1.5;
auv.ctrl.theta.Kd   = 0.8;
auv.ctrl.theta.sat  = deg2rad(15);   % Output saturation (rad)

% Heading / yaw — with 3rd-order LP pre-filter
auv.ctrl.psi.Kp     = 3.0;
auv.ctrl.psi.Ki     = 1.0;
auv.ctrl.psi.Kd     = 0.5;
auv.ctrl.psi.sat    = deg2rad(15);   % Output saturation (rad)
auv.ctrl.psi.omega_filter = 0.5;    % LP filter bandwidth (rad/s)

% =========================================================================
% 9.  Guidance parameters  (Phase 6)
% =========================================================================
auv.guid.delta_y    = 150;     % LOS horizontal lookahead (m)
auv.guid.delta_z    = 150;     % LOS vertical lookahead (m)
auv.guid.k_y        = 1;       % LOS lateral gain
auv.guid.k_z        = 1;       % LOS vertical gain
auv.guid.U_max      = 2.5;     % Maximum guidance-commanded speed (m/s)
auv.guid.U_min      = 0.3;     % Minimum speed (stall avoidance)

% =========================================================================
% 10.  Navigation and sensor noise  (Phase 4)
% =========================================================================
% Set to zero during Phase 4 pass-through, increase later
auv.nav.sigma_pos   = 0.0;     % Position noise std dev (m)
auv.nav.sigma_vel   = 0.0;     % Velocity noise std dev (m/s)
auv.nav.sigma_angle = 0.0;     % Angle noise std dev (rad)
auv.nav.sigma_rate  = 0.0;     % Angular rate noise std dev (rad/s)

% =========================================================================
% Report
% =========================================================================
fprintf('\n=== AUV Parameters Loaded ===\n');
fprintf('  Vehicle:   L=%.1fm, D=%.2fm, m=%.1fkg\n', ...
    auv.phys.L, auv.phys.D, auv.phys.m);
fprintf('  Speed max: %.1f m/s at %d RPM\n', 2.5, auv.act.n_max);
fprintf('  Fin limit: ±%.0f deg\n', rad2deg(auv.act.delta_max));
fprintf('  IC:        x0 = zeros(12,1)\n');
fprintf('  Ts:        %.3f s (%.0f Hz)\n', auv.sim.Ts, 1/auv.sim.Ts);
fprintf('\nParameter struct ''auv'' assigned to base workspace.\n\n');

assignin('base', 'auv', auv);
