%% logger_lib.m  —  Signal Logger Library  (Phase 8)
%
% PURPOSE:
%   Structured data capture for all simulation signals.
%   The logger is a pure read-only tap — it never writes to any
%   signal and has no effect on simulation results.
%
% DESIGN:
%   Pre-allocated fixed-size arrays (not growing cell arrays) for
%   real-time performance. All signals are indexed by step number k.
%   A single 'log' struct is passed by reference and returned updated.
%
% LOG STRUCT FIELDS:
%   log.t          [1×N]    simulation time vector (s)
%   log.x          [12×N]   full true state
%   log.nu_hat     [6×N]    estimated velocities from Navigation
%   log.eta_hat    [6×N]    estimated position/attitude from Navigation
%   log.tau_ctrl   [6×N]    generalised forces from Control
%   log.ui         [3×N]    physical actuator commands
%   log.pwm        [3×N]    PWM values (µs)
%   log.sat_flags  [3×N]    saturation flags
%   log.n_direct   [1×N]    direct RPM command
%   log.e_chi      [1×N]    heading error (rad)
%   log.e_theta    [1×N]    pitch error (rad)
%   log.e_u        [1×N]    speed error (m/s)
%   log.Vc         [1×N]    ocean current speed
%   log.betaVc     [1×N]    ocean current direction
%   log.tau_env    [6×N]    environmental forces
%   log.chi_d      [1×N]    desired course angle
%   log.upsilon_d  [1×N]    desired flight-path angle
%   log.ud         [1×N]    desired speed
%   log.los_xe     [1×N]    along-track error
%   log.los_ye     [1×N]    lateral cross-track error
%   log.los_ze     [1×N]    vertical cross-track error
%   log.N          scalar   number of steps allocated
%   log.k          scalar   current write index
%
% FUNCTIONS:
%   log_init        — allocate log struct for N steps
%   log_step        — record one timestep
%   log_trim        — trim to actual recorded length
%   log_save        — save to .mat file with timestamp
%   log_summary     — print key statistics to console
%
% AUTHOR: AUV Simulation Project — Phase 8

% =========================================================================
function log = log_init(N)
%% log_init  —  Pre-allocate log struct for N timesteps

log.t         = zeros(1, N);
log.x         = zeros(12, N);
log.nu_hat    = zeros(6, N);
log.eta_hat   = zeros(6, N);
log.tau_ctrl  = zeros(6, N);
log.ui        = zeros(3, N);
log.pwm       = zeros(3, N);
log.sat_flags = zeros(3, N, 'uint8');
log.n_direct  = zeros(1, N);
log.e_chi     = zeros(1, N);
log.e_theta   = zeros(1, N);
log.e_u       = zeros(1, N);
log.Vc        = zeros(1, N);
log.betaVc    = zeros(1, N);
log.tau_env   = zeros(6, N);
log.chi_d     = zeros(1, N);
log.upsilon_d = zeros(1, N);
log.ud        = zeros(1, N);
log.los_xe    = zeros(1, N);
log.los_ye    = zeros(1, N);
log.los_ze    = zeros(1, N);
log.N         = N;
log.k         = 0;

end

% =========================================================================
function log = log_step(log, t, x, nav, ctrl, act, env, guid)
%% log_step  —  Record one timestep into the log struct
%
% INPUTS:
%   t     scalar    simulation time (s)
%   x     [12×1]   true state from Dynamics
%   nav   struct    Navigation outputs: nu_hat, eta_hat
%   ctrl  struct    Control debug: tau_ctrl, n_direct, e_chi, e_theta, e_u
%   act   struct    Actuation outputs: ui, pwm, sat_flags
%   env   struct    Environment: Vc, betaVc, tau_env
%   guid  struct    Guidance: chi_d, upsilon_d, ud, los_errors
%
% All inputs are optional — missing fields silently leave zeros.
% This means log_step works even before all modules are complete.

k = log.k + 1;
if k > log.N
    warning('log_step: log is full (k=%d > N=%d). Discarding.', k, log.N);
    return
end

log.k           = k;
log.t(k)        = t;
log.x(:,k)      = x;

if isfield(nav,'nu_hat'),   log.nu_hat(:,k)  = nav.nu_hat;   end
if isfield(nav,'eta_hat'),  log.eta_hat(:,k) = nav.eta_hat;  end

if isfield(ctrl,'tau_ctrl'), log.tau_ctrl(:,k) = ctrl.tau_ctrl; end
if isfield(ctrl,'n_direct'), log.n_direct(k)   = ctrl.n_direct; end
if isfield(ctrl,'e_chi'),    log.e_chi(k)       = ctrl.e_chi;    end
if isfield(ctrl,'e_theta'),  log.e_theta(k)     = ctrl.e_theta;  end
if isfield(ctrl,'e_u'),      log.e_u(k)         = ctrl.e_u;      end

if isfield(act,'ui'),        log.ui(:,k)         = act.ui;        end
if isfield(act,'pwm'),       log.pwm(:,k)        = act.pwm;       end
if isfield(act,'sat_flags'), log.sat_flags(:,k)  = act.sat_flags; end

if isfield(env,'Vc'),        log.Vc(k)           = env.Vc;        end
if isfield(env,'betaVc'),    log.betaVc(k)       = env.betaVc;    end
if isfield(env,'tau_env'),   log.tau_env(:,k)    = env.tau_env;   end

if isfield(guid,'chi_d'),    log.chi_d(k)        = guid.chi_d;    end
if isfield(guid,'upsilon_d'),log.upsilon_d(k)    = guid.upsilon_d;end
if isfield(guid,'ud'),       log.ud(k)           = guid.ud;       end
if isfield(guid,'los_xe'),   log.los_xe(k)       = guid.los_xe;   end
if isfield(guid,'los_ye'),   log.los_ye(k)       = guid.los_ye;   end
if isfield(guid,'los_ze'),   log.los_ze(k)       = guid.los_ze;   end

end

% =========================================================================
function log = log_trim(log)
%% log_trim  —  Trim all arrays to the actual number of recorded steps

k = log.k;
if k == 0
    warning('log_trim: no data recorded.');
    return
end

fields = fieldnames(log);
for i = 1:numel(fields)
    f = fields{i};
    if strcmp(f,'N') || strcmp(f,'k'), continue, end
    v = log.(f);
    if isvector(v) && numel(v) == log.N
        log.(f) = v(1:k);
    elseif ~isvector(v) && size(v,2) == log.N
        log.(f) = v(:, 1:k);
    end
end

log.N = k;

end

% =========================================================================
function filename = log_save(log, prefix)
%% log_save  —  Save log to timestamped .mat file
%
% INPUTS:
%   log      log struct (trimmed or full)
%   prefix   string file prefix (e.g. 'phase8', 'run_001')

if nargin < 2, prefix = 'auv_log'; end

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
filename  = sprintf('%s_%s.mat', prefix, timestamp);
save(filename, 'log');
fprintf('Log saved: %s  (%d steps, %.1f s)\n', filename, log.k, log.t(log.k));

end

% =========================================================================
function log_summary(log)
%% log_summary  —  Print key statistics to console

if log.k == 0
    fprintf('Log is empty.\n');
    return
end

k = log.k;
t_end = log.t(k);

fprintf('\n=== Simulation Log Summary ===\n');
fprintf('  Duration:      %.1f s  (%d steps)\n', t_end, k);

% State bounds
u_max = max(abs(log.x(1,1:k)));
v_max = max(abs(log.x(2,1:k)));
w_max = max(abs(log.x(3,1:k)));
fprintf('  Max |u|:       %.3f m/s\n', u_max);
fprintf('  Max |v|:       %.3f m/s\n', v_max);
fprintf('  Max |w|:       %.3f m/s\n', w_max);

% Position range
x_range = [min(log.x(7,1:k)), max(log.x(7,1:k))];
y_range = [min(log.x(8,1:k)), max(log.x(8,1:k))];
z_range = [min(log.x(9,1:k)), max(log.x(9,1:k))];
fprintf('  North range:   [%.1f, %.1f] m\n', x_range(1), x_range(2));
fprintf('  East  range:   [%.1f, %.1f] m\n', y_range(1), y_range(2));
fprintf('  Depth range:   [%.2f, %.2f] m\n', z_range(1), z_range(2));

% Control performance
if any(log.e_chi(1:k) ~= 0)
    rms_chi = sqrt(mean(log.e_chi(1:k).^2));
    rms_u   = sqrt(mean(log.e_u(1:k).^2));
    fprintf('  RMS heading error: %.2f deg\n', rad2deg(rms_chi));
    fprintf('  RMS speed error:   %.3f m/s\n', rms_u);
end

% Cross-track errors
if any(log.los_ye(1:k) ~= 0)
    rms_ye = sqrt(mean(log.los_ye(1:k).^2));
    rms_ze = sqrt(mean(log.los_ze(1:k).^2));
    fprintf('  RMS lateral x-track:   %.2f m\n', rms_ye);
    fprintf('  RMS vertical x-track:  %.2f m\n', rms_ze);
end

% Saturation events
sat_total = sum(log.sat_flags(:,1:k) > 0, 'all');
fprintf('  Saturation events: %d\n', sat_total);

% NaN / Inf check
if any(isnan(log.x(:,1:k)),'all') || any(isinf(log.x(:,1:k)),'all')
    fprintf('  WARNING: NaN or Inf detected in state log!\n');
else
    fprintf('  State integrity: OK (no NaN/Inf)\n');
end

fprintf('==============================\n\n');

end
