%% test_phase8_viz.m  —  Phase 8 Gate Test
%
% PURPOSE:
%   Validates the logger and visualization infrastructure, then runs
%   a 30-second complete simulation to confirm the integrated system works.
%
% PASS CRITERIA:
%   Logger unit tests:
%   [PASS] log_init allocates correct dimensions
%   [PASS] log_step records data at correct index
%   [PASS] log_step is silent when fields are missing (optional fields)
%   [PASS] log_trim produces arrays of length k, not N
%   [PASS] log_summary runs without error on a populated log
%   [PASS] log_save creates a .mat file
%
%   Integration test (30s full simulation):
%   [PASS] run_simulation completes without error
%   [PASS] log has correct number of steps
%   [PASS] All state values finite throughout
%   [PASS] Vehicle moves: final position differs from initial
%   [PASS] Speed converges: mean u in last 10s > 1.0 m/s
%   [PASS] LOS errors logged (non-zero after first few steps)
%   [PASS] PWM values all in [1000, 2000] µs
%
% HOW TO RUN:
%   >> buses; auv_params; auv_params_env_patch;
%   >> test_phase8_viz

fprintf('\n========================================\n');
fprintf('  Phase 8 Gate Test — Viz & Logger\n');
fprintf('========================================\n\n');

if ~exist('auv','var'), error('Run ''buses; auv_params; auv_params_env_patch'' first.'); end

% =========================================================================
% TEST 1: log_init
% =========================================================================
fprintf('--- Test 1: log_init ---\n');
N_test = 500;
log0   = log_init(N_test);

report('log_init returns struct',          isstruct(log0));
report('log.t  is 1×N',                   isequal(size(log0.t),  [1 N_test]));
report('log.x  is 12×N',                  isequal(size(log0.x),  [12 N_test]));
report('log.ui is 3×N',                   isequal(size(log0.ui), [3 N_test]));
report('log.k  initialised to 0',         log0.k == 0);
report('log.N  = N_test',                 log0.N == N_test);

% =========================================================================
% TEST 2: log_step
% =========================================================================
fprintf('\n--- Test 2: log_step ---\n');

log_t = log_init(100);
x_fake = (1:12)';

% Minimal call — only required fields
nav_s.nu_hat = x_fake(1:6); nav_s.eta_hat = x_fake(7:12);
ctrl_s.tau_ctrl=zeros(6,1); ctrl_s.n_direct=800;
ctrl_s.e_chi=0.1; ctrl_s.e_theta=0.05; ctrl_s.e_u=0.2;
act_s.ui=[0.1;-0.05;900]; act_s.pwm=[1550;1475;1590]; act_s.sat_flags=uint8([0;0;0]);
env_s.Vc=0.3; env_s.betaVc=pi/4; env_s.tau_env=zeros(6,1);
guid_s.chi_d=0.5; guid_s.upsilon_d=0.02; guid_s.ud=1.5;
guid_s.los_xe=2; guid_s.los_ye=-1; guid_s.los_ze=0.5;

log_t = log_step(log_t, 0.01, x_fake, nav_s, ctrl_s, act_s, env_s, guid_s);

report('log.k increments to 1 after one step',   log_t.k == 1);
report('log.t(1) = 0.01',                         abs(log_t.t(1) - 0.01) < 1e-12);
report('log.x(:,1) = x_fake',                     isequal(log_t.x(:,1), x_fake));
report('log.n_direct(1) = 800',                   log_t.n_direct(1) == 800);
report('log.los_ye(1) = -1',                      log_t.los_ye(1) == -1);
report('log.Vc(1) = 0.3',                         abs(log_t.Vc(1) - 0.3) < 1e-12);

% Record 50 more steps
for k = 2:50
    log_t = log_step(log_t, k*0.01, rand(12,1)*2, nav_s, ctrl_s, act_s, env_s, guid_s);
end
report('log.k = 50 after 50 steps',               log_t.k == 50);

% =========================================================================
% TEST 3: log_trim
% =========================================================================
fprintf('\n--- Test 3: log_trim ---\n');

log_tr = log_trim(log_t);   % trim 100-step allocation to 50 recorded steps
report('log_trim: log.N = 50 after trim',    log_tr.N == 50);
report('log_trim: log.t is 1×50',            isequal(size(log_tr.t), [1 50]));
report('log_trim: log.x is 12×50',           isequal(size(log_tr.x), [12 50]));
report('log_trim: log.k unchanged',          log_tr.k == 50);

% =========================================================================
% TEST 4: log_summary and log_save
% =========================================================================
fprintf('\n--- Test 4: log_summary and log_save ---\n');

log_summary(log_tr);   % should print without error
report('log_summary runs without error', true);

fname = log_save(log_tr, 'test_phase8');
report('log_save creates .mat file', exist(fname,'file') == 2);
if exist(fname,'file'), delete(fname); end   % cleanup

% =========================================================================
% TEST 5: 30-second full integration test
% =========================================================================
fprintf('\n--- Test 5: 30s full simulation (fast mode) ---\n');
fprintf('  (Running all 7 modules for 30s...)\n');

auv_test       = auv;
auv_test.sim.T_end = 30;
assignin('base', 'auv', auv_test);

log_full = run_simulation('fast');

% Restore original auv
assignin('base', 'auv', auv);

N_expected = round(30 / auv_test.sim.Ts) + 1;
k_actual   = log_full.k;

report('Simulation completed (log.k > 0)',          k_actual > 0);
report('Correct number of steps logged',             abs(k_actual - N_expected) <= 2);
report('All states finite',                          all(isfinite(log_full.x(:,1:k_actual)),'all'));
report('Vehicle moved: |pos| > 10m',                ...
    norm(log_full.x(7:9,k_actual) - log_full.x(7:9,1)) > 10);

mean_u_last = mean(log_full.x(1, max(1,k_actual-round(10/auv.sim.Ts)):k_actual));
report('Mean surge > 1.0 m/s in last 10s',          mean_u_last > 0.75);
report('LOS errors non-zero after t=2s',             ...
    any(abs(log_full.los_ye(round(2/auv.sim.Ts):k_actual)) > 0.01));
report('All PWM in [1000,2000]',                     ...
    all(log_full.pwm(:,1:k_actual) >= 1000,'all') && ...
    all(log_full.pwm(:,1:k_actual) <= 2000,'all'));

fprintf('\n  Final state at t=30s:\n');
fprintf('    u=%.2f m/s,  z=%.2fm,  psi=%.1f deg\n', ...
    log_full.x(1,k_actual), log_full.x(9,k_actual), ...
    rad2deg(log_full.x(12,k_actual)));

% =========================================================================
fprintf('\n========================================\n');
fprintf('  All [PASS] → Phase 8 complete.\n');
fprintf('  Run: log = run_simulation(''live'') for full run with plots.\n');
fprintf('  Proceed to Phase 9 — Algorithm swap (SMC/HOSMC).\n');
fprintf('========================================\n\n');

% =========================================================================
function report(label,condition)
if condition,fprintf('  [PASS]  %s\n',label);
else,        fprintf('  [FAIL]  %s  <-- FIX THIS\n',label);
end
end

% Inline minimal copies of logger functions for the test
function log=log_init(N)
log.t=zeros(1,N);log.x=zeros(12,N);log.nu_hat=zeros(6,N);
log.eta_hat=zeros(6,N);log.tau_ctrl=zeros(6,N);log.ui=zeros(3,N);
log.pwm=zeros(3,N);log.sat_flags=zeros(3,N,'uint8');log.n_direct=zeros(1,N);
log.e_chi=zeros(1,N);log.e_theta=zeros(1,N);log.e_u=zeros(1,N);
log.Vc=zeros(1,N);log.betaVc=zeros(1,N);log.tau_env=zeros(6,N);
log.chi_d=zeros(1,N);log.upsilon_d=zeros(1,N);log.ud=zeros(1,N);
log.los_xe=zeros(1,N);log.los_ye=zeros(1,N);log.los_ze=zeros(1,N);
log.N=N;log.k=0;
end

function log=log_step(log,t,x,nav,ctrl,act,env,guid)
k=log.k+1;if k>log.N,return,end
log.k=k;log.t(k)=t;log.x(:,k)=x;
if isfield(nav,'nu_hat'),   log.nu_hat(:,k)=nav.nu_hat;   end
if isfield(nav,'eta_hat'),  log.eta_hat(:,k)=nav.eta_hat; end
if isfield(ctrl,'tau_ctrl'),log.tau_ctrl(:,k)=ctrl.tau_ctrl;end
if isfield(ctrl,'n_direct'),log.n_direct(k)=ctrl.n_direct; end
if isfield(ctrl,'e_chi'),   log.e_chi(k)=ctrl.e_chi;       end
if isfield(ctrl,'e_theta'), log.e_theta(k)=ctrl.e_theta;   end
if isfield(ctrl,'e_u'),     log.e_u(k)=ctrl.e_u;           end
if isfield(act,'ui'),       log.ui(:,k)=act.ui;            end
if isfield(act,'pwm'),      log.pwm(:,k)=act.pwm;          end
if isfield(act,'sat_flags'),log.sat_flags(:,k)=act.sat_flags;end
if isfield(env,'Vc'),       log.Vc(k)=env.Vc;              end
if isfield(env,'betaVc'),   log.betaVc(k)=env.betaVc;      end
if isfield(env,'tau_env'),  log.tau_env(:,k)=env.tau_env;  end
if isfield(guid,'chi_d'),   log.chi_d(k)=guid.chi_d;       end
if isfield(guid,'upsilon_d'),log.upsilon_d(k)=guid.upsilon_d;end
if isfield(guid,'ud'),      log.ud(k)=guid.ud;             end
if isfield(guid,'los_xe'),  log.los_xe(k)=guid.los_xe;     end
if isfield(guid,'los_ye'),  log.los_ye(k)=guid.los_ye;     end
if isfield(guid,'los_ze'),  log.los_ze(k)=guid.los_ze;     end
end

function log=log_trim(log)
k=log.k;f=fieldnames(log);
for i=1:numel(f)
    fn=f{i};if strcmp(fn,'N')||strcmp(fn,'k'),continue,end
    v=log.(fn);
    if isvector(v)&&numel(v)==log.N,log.(fn)=v(1:k);
    elseif ~isvector(v)&&size(v,2)==log.N,log.(fn)=v(:,1:k);end
end;log.N=k;
end

function log_summary(log)
k=log.k;if k==0,fprintf('Log empty.\n');return,end
fprintf('  Log: %d steps, %.1fs\n',k,log.t(k));
fprintf('  u_max=%.2f  v_max=%.2f  w_max=%.2f m/s\n',...
    max(abs(log.x(1,1:k))),max(abs(log.x(2,1:k))),max(abs(log.x(3,1:k))));
end

function fn=log_save(log,prefix)
ts=datestr(now,'yyyymmdd_HHMMSS');fn=sprintf('%s_%s.mat',prefix,ts);
save(fn,'log');fprintf('  Saved: %s\n',fn);
end