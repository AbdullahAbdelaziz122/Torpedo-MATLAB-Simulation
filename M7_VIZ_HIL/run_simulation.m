%% run_simulation.m  —  Complete AUV Simulation Runner  (Phase 8)
%
% PURPOSE:
%   Runs the full closed-loop AUV simulation integrating all seven modules
%   (M1-M7) and adding logging and real-time visualization (Phase 8).
%   This is the single entry point for the complete system.
%
% MODULES INTEGRATED:
%   M1 — Environment    (environment_lib.m)
%   M2 — Guidance       (guidance_los_lib.m + path_lib.m)
%   M3 — Control        (control_pid_lib.m)
%   M4 — Actuation      (actuation_lib.m)
%   M5 — Dynamics       (remus100.m, MSS Toolbox)
%   M6 — Navigation     (navigation_lib.m)
%   M7 — Viz/Log/HIL    (viz_lib.m + logger_lib.m)
%
% USAGE:
%   >> buses; auv_params; auv_params_env_patch;
%   >> run_simulation           % full run, live plots, auto-save log
%   >> run_simulation('quiet')  % no live plots, just final figures
%   >> run_simulation('fast')   % no plots at all, maximum speed
%
% OUTPUT:
%   log    — complete signal log struct in workspace
%   Saved  — auv_log_TIMESTAMP.mat in current directory
%
% AUTHOR: AUV Simulation Project — Phase 8

function log = run_simulation(mode)

if nargin < 1, mode = 'live'; end

% =========================================================================
% Setup
% =========================================================================
if evalin('base', 'exist(''auv'',''var'')') == 0
    error('Run ''buses; auv_params; auv_params_env_patch'' first.');
end
auv_sim = evalin('base','auv');

live_plots = strcmp(mode,'live');
show_final = ~strcmp(mode,'fast');

fprintf('\n=== AUV Simulation — Phase 8 ===\n');
fprintf('  Mode:     %s\n', mode);
fprintf('  Duration: %.0f s\n', auv_sim.sim.T_end);
fprintf('  Ts:       %.3f s  (%.0f Hz)\n', auv_sim.sim.Ts, 1/auv_sim.sim.Ts);

% =========================================================================
% Initialise all modules
% =========================================================================
dt     = auv_sim.sim.Ts;
T_end  = auv_sim.sim.T_end;
N      = round(T_end / dt) + 1;
tvec   = 0 : dt : T_end;

% M1 — Environment
env_state = env_init_local(auv_sim);

% M2 — Path + Guidance
path   = path_helix_local(auv_sim);
los    = los_init_local(auv_sim);

% M3 — Control
cs     = ctrl_init_local(auv_sim);

% M6 — Navigation (pass-through in Phase 8)
% No init needed — stateless pass-through

% M7 — Logger + Viz
log    = log_init(N);
if live_plots
    viz = viz_init(50);   % update every 50 steps = 0.5 s
end

% Initial state
x0          = zeros(12,1);
x0(1)       = 1.5;                   % start at cruise speed
x0(7)       = path.pts(1,1);         % align with path start
x0(8)       = path.pts(2,1);
x0(9)       = path.pts(3,1);
x0(12)      = path.psi0;             % heading aligned with path

x = x0;

% =========================================================================
% Simulation loop
% =========================================================================
fprintf('\n  Running...\n');
tic;

for k = 1:N
    t = tvec(k);

    % --- M6: Navigation (pass-through) ---
    nu_hat  = x(1:6);
    eta_hat = x(7:12);
    eta_hat(4:6) = arrayfun(@(a) atan2(sin(a),cos(a)), eta_hat(4:6));

    % --- M1: Environment ---
    [Vc, betaVc, w_c, tau_env, env_state] = env_step_local(env_state, t);

    % --- M2: Path query + LOS Guidance ---
    [pt, pt_vel] = path_query_local(path, t);
    desire_spd   = [auv_sim.guid.U_min; 0; 0];
    [chi_d, upsilon_d, ud, ~, ~, errs] = ...
        los_step_local(nu_hat, eta_hat, pt, pt_vel, desire_spd, los);

    guid.chi_d     = chi_d;
    guid.upsilon_d = upsilon_d;
    guid.ud        = ud;
    guid.z_des     = pt(3);
    guid.los_xe    = errs.x_e;
    guid.los_ye    = errs.y_e;
    guid.los_ze    = errs.z_e;

    % --- M3: Control (PID) ---
    [tau_ctrl, n_direct, ctrl_dbg, cs] = ctrl_step_local(guid, nu_hat, eta_hat, cs);

    % --- M4: Actuation ---
    [ui, sat_flags] = actuation_local(tau_ctrl, n_direct, nu_hat(1), auv_sim);
    pwm             = ui_to_pwm_local(ui, auv_sim);

    % --- M7: Log ---
    nav_log.nu_hat   = nu_hat;
    nav_log.eta_hat  = eta_hat;

    ctrl_log.tau_ctrl = tau_ctrl;
    ctrl_log.n_direct = n_direct;
    ctrl_log.e_chi    = ctrl_dbg.e_chi;
    ctrl_log.e_theta  = ctrl_dbg.e_theta;
    ctrl_log.e_u      = ctrl_dbg.e_u;

    act_log.ui        = ui;
    act_log.pwm       = pwm;
    act_log.sat_flags = sat_flags;

    env_log.Vc        = Vc;
    env_log.betaVc    = betaVc;
    env_log.tau_env   = tau_env;

    log = log_step(log, t, x, nav_log, ctrl_log, act_log, env_log, guid);

    % --- M7: Live visualization ---
    if live_plots && mod(k, viz.update_every) == 0
        viz = viz_update(viz, log);
    end

    % --- M5: Dynamics (RK4) ---
    if k < N
        x = rk4_step_local(x, ui, Vc, betaVc, w_c, tau_env, dt, auv_sim);
    end

    % Progress indicator every 10s
    if mod(k-1, round(10/dt)) == 0 && k > 1
        elapsed = toc;
        fprintf('  t = %5.1f s  |  u = %.2f m/s  |  z = %.2f m  |  wall = %.1fs\n', ...
            t, x(1), x(9), elapsed);
    end
end

sim_time = toc;

% =========================================================================
% Finalise
% =========================================================================
log = log_trim(log);
log_summary(log);

filename = log_save(log, 'auv_log');
fprintf('  Total wall time: %.1f s  (%.1fx real-time)\n\n', ...
    sim_time, T_end/sim_time);

if live_plots
    viz_close(viz);   % close live figures before opening post-run set
end

if show_final
    viz_final(log);
end

assignin('base', 'log', log);
fprintf('  ''log'' struct available in workspace.\n');
fprintf('  Run viz_final(log) to regenerate figures.\n\n');

end

% =========================================================================
% RK4 step — fixed-step 4th-order Runge-Kutta
% =========================================================================
function x_next = rk4_step_local(x, ui, Vc, betaVc, w_c, tau_env, dt, auv)
%% RK4 integration of remus100.m with external tau_env injection
%
% tau_env is injected at the acceleration level: a_env = M\tau_env
% This is physically correct — external forces act directly on the hull.
%
% The RK4 scheme evaluates the dynamics function at 4 points:
%   k1 = f(x)
%   k2 = f(x + dt/2 * k1)
%   k3 = f(x + dt/2 * k2)
%   k4 = f(x + dt * k3)
%   x_next = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4)

% Get mass matrix once (constant for submerged AUV)
[~, ~, M] = remus100();
a_env     = [M \ tau_env(1:6); zeros(6,1)];   % acceleration injection

f = @(xx) remus100(xx, ui, Vc, betaVc, w_c) + a_env;

k1 = f(x);
k2 = f(x + (dt/2)*k1);
k3 = f(x + (dt/2)*k2);
k4 = f(x + dt*k3);

x_next = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);

end

% =========================================================================
% Condensed module copies — canonical sources are the _lib.m files
% =========================================================================

function es=env_init_local(auv)
if ~isfield(auv,'env'), es=env_zero(); return, end
e=auv.env; es.Vc=e.Vc_mean;es.betaVc=e.betaVc_mean;es.w_c=e.w_c;
es.Vc_gm=e.Vc_mean;es.beta_gm=e.betaVc_mean;es.wave_x=zeros(6,1);
es.wave_on=e.wave_on;es.dt=auv.sim.Ts;
es.mu_Vc=e.mu_Vc;es.mu_beta=e.mu_betaVc;
es.sigma_Vc=e.sigma_Vc;es.sigma_beta=e.sigma_betaVc;
if e.wave_on&&e.Hs>0
    es.wf_Z=wf_i(e.Hs,e.Tp,e.wave_scale_Z);
    es.wf_K=wf_i(e.Hs*0.3,e.Tp,e.wave_scale_K);
    es.wf_M=wf_i(e.Hs*0.5,e.Tp,e.wave_scale_M);
else
    es.wf_Z=wf_i(0,6,0);es.wf_K=wf_i(0,6,0);es.wf_M=wf_i(0,6,0);
end
end

function wf=wf_i(Hs,Tp,sc)
if Hs<=0||Tp<=0,wf.omega_n=1;wf.zeta=0.1;wf.Kw=0;return,end
wf.omega_n=2*pi/Tp;wf.zeta=0.1;wf.Kw=sc*(Hs/4)*sqrt(2*wf.zeta*wf.omega_n);
end

function es=env_zero()
es.Vc=0;es.betaVc=0;es.w_c=0;es.Vc_gm=0;es.beta_gm=0;es.wave_x=zeros(6,1);
es.wave_on=false;es.dt=0.01;es.mu_Vc=0;es.mu_beta=0;es.sigma_Vc=0;es.sigma_beta=0;
es.wf_Z=wf_i(0,6,0);es.wf_K=wf_i(0,6,0);es.wf_M=wf_i(0,6,0);
end

function [Vc,bVc,wc,tau_env,es]=env_step_local(es,t) %#ok<INUSL>
dt=es.dt;
es.Vc_gm  =gm_step(es.Vc_gm,  es.mu_Vc,  es.sigma_Vc,  dt);
es.beta_gm=gm_step(es.beta_gm,es.mu_beta,es.sigma_beta,dt);
Vc=max(0,es.Vc_gm);bVc=atan2(sin(es.beta_gm),cos(es.beta_gm));wc=es.w_c;
es.Vc=Vc;es.betaVc=bVc;tau_env=zeros(6,1);
if es.wave_on
    [tau_env(3),es.wave_x(1:2)]=wf_step(es.wave_x(1:2),es.wf_Z,dt);
    [tau_env(4),es.wave_x(3:4)]=wf_step(es.wave_x(3:4),es.wf_K,dt);
    [tau_env(5),es.wave_x(5:6)]=wf_step(es.wave_x(5:6),es.wf_M,dt);
end
end

function [o,xn]=wf_step(x,wf,dt)
w=randn;dx1=x(2);dx2=-wf.omega_n^2*x(1)-2*wf.zeta*wf.omega_n*x(2)+wf.Kw*w;
xn=[x(1)+dt*dx1;x(2)+dt*dx2];o=xn(1);
end

function xn=gm_step(x,mu,sigma,dt)
if sigma<=0,xn=x;return,end;xn=x+dt*(-mu*x+sigma*randn);
end

function path=path_helix_local(auv)
T=auv.sim.T_end;dt=auv.sim.Ts;time=0:dt:T;m=0.6;Yr=m*time;
X=60*cos(0.02618*Yr);Y=60*sin(0.02618*Yr);Z=2+(2*Yr/200);
dX=-60*0.02618*m*sin(0.02618*Yr);dY=60*0.02618*m*cos(0.02618*Yr);
dZ=2*m/200*ones(size(time));
path.type='helix';path.t_vec=time;path.pts=[X;Y;Z];path.vel=[dX;dY;dZ];
path.T_total=T;path.U_nom=m;path.psi0=atan2(dY(1),dX(1));
path.len=sum(sqrt(diff(X).^2+diff(Y).^2+diff(Z).^2));
end

function [pt,vel]=path_query_local(path,t)
t=max(0,min(t,path.T_total));
idx=max(1,min(round(t/(path.t_vec(2)-path.t_vec(1)))+1,size(path.pts,2)));
pt=path.pts(:,idx);vel=path.vel(:,idx);
end

function los=los_init_local(auv)
los.ky=auv.guid.k_y;los.kz=auv.guid.k_z;
los.delta_y=auv.guid.delta_y;los.delta_z=auv.guid.delta_z;
los.U_max=auv.guid.U_max;los.U_min=auv.guid.U_min;los.U_nom=1.5;
end

function [cd,ud_a,ud_s,cv,uv,errs]=los_step_local(nu,eta,pt,pv,ds,los)
psi_v=eta(6);Ry=[cos(psi_v) -sin(psi_v) 0;sin(psi_v) cos(psi_v) 0;0 0 1];
vn=Ry*nu(1:3);
if norm(vn(1:2))<1e-4,cv=0;else,cv=atan2(vn(2),vn(1));end
h=sqrt(vn(1)^2+vn(2)^2);if h<1e-4,uv=0;else,uv=atan(-vn(3)/h);end
if norm(pv(1:2))<1e-6,pp=0;else,pp=atan2(pv(2),pv(1));end
h2=sqrt(pv(1)^2+pv(2)^2);if h2<1e-6,tp=0;else,tp=atan(-pv(3)/h2);end
dx=eta(1)-pt(1);dy=eta(2)-pt(2);dz=eta(3)-pt(3);
cp=cos(pp);sp=sin(pp);ct=cos(tp);st=sin(tp);
xe=cp*ct*dx+sp*ct*dy-st*dz;ye=-sp*dx+cp*dy;ze=cp*st*dx+sp*st*dy+ct*dz;
pr=tanh(-los.ky*ye/los.delta_y);tr=tanh(los.kz*ze/los.delta_z);
arg=max(-1+1e-12,min(1-1e-12,sin(tp)*cos(tr)*cos(pr)+cos(tp)*sin(tr)));
ud_a=asin(arg);
cy=cos(pp)*sin(pr)*cos(tr)-sin(tp)*sin(tr)*sin(pp)+sin(pp)*cos(pr)*cos(tp)*cos(tr);
cx=-sin(pp)*sin(pr)*cos(tr)-sin(tp)*sin(tr)*cos(pp)+cos(pp)*cos(pr)*cos(tp)*cos(tr);
cd=atan2(cy,cx);
denom=max(cos(pr)*cos(tr),0.1);raw=(norm(pv)-0.001*xe)/denom;
u=ds(1);v=ds(2);w=ds(3);mag=sqrt(u^2+v^2+w^2);
if mag<1e-6,ud_s=los.U_nom;else,ud_s=raw*u/mag;end
ud_s=max(los.U_min,min(los.U_max,ud_s));
errs.x_e=xe;errs.y_e=ye;errs.z_e=ze;
end

function cs=ctrl_init_local(auv)
cs.dt=auv.sim.Ts;
cs.u.integral=0;cs.u.e_prev=0;cs.u.Kp=auv.ctrl.u.Kp;
cs.u.Ki=auv.ctrl.u.Ki;cs.u.Kd=auv.ctrl.u.Kd;cs.u.sat=auv.ctrl.u.u_max;
cs.z.integral=0;cs.z.Kp=auv.ctrl.z.Kp;cs.z.Ki=auv.ctrl.z.Ki;
cs.z.theta_max=auv.ctrl.z.theta_d_max;
cs.theta.integral=0;cs.theta.e_prev=0;cs.theta.Kp=auv.ctrl.theta.Kp;
cs.theta.Ki=auv.ctrl.theta.Ki;cs.theta.Kd=auv.ctrl.theta.Kd;cs.theta.sat=auv.ctrl.theta.sat;
cs.psi.integral=0;cs.psi.e_prev=0;cs.psi.Kp=auv.ctrl.psi.Kp;
cs.psi.Ki=auv.ctrl.psi.Ki;cs.psi.Kd=auv.ctrl.psi.Kd;cs.psi.sat=auv.ctrl.psi.sat;
[~,~,M]=remus100();cs.m11=M(1,1);cs.m55=M(5,5);cs.m66=M(6,6);
cs.m35=M(3,5);cs.m26=M(2,6);cs.W=auv.phys.W;cs.B=auv.phys.B;
cs.zg=auv.phys.r_bG(3);cs.zb=auv.phys.r_bB(3);
end

function [tc,nd,dbg,cs]=ctrl_step_local(guid,nu,eta,cs)
u=nu(1);v=nu(2);w=nu(3);q=nu(5);r=nu(6);theta=eta(5);
chi_v=atan2(sin(eta(6)),cos(eta(6)));Ts=cs.dt;
[~,~,M]=remus100();m55=M(5,5);m66=M(6,6);m35=M(3,5);m26=M(2,6);
W=cs.W;B=cs.B;zg=cs.zg;zb=cs.zb;
e_u=guid.ud-u;du=(e_u-cs.u.e_prev)/Ts;
out_u=cs.u.Kp*e_u+cs.u.Ki*cs.u.integral+cs.u.Kd*du;
nd=max(0,min(1525,(1525/20)*out_u));
if abs(out_u)<=cs.u.sat,cs.u.integral=cs.u.integral+e_u*Ts;end;cs.u.e_prev=e_u;
z_d=eta(3);e_z=guid.z_des-z_d;
out_z=cs.z.Kp*e_z+cs.z.Ki*cs.z.integral;
theta_d=max(-cs.z.theta_max,min(cs.z.theta_max,out_z));
if abs(out_z)<=cs.z.theta_max,cs.z.integral=cs.z.integral+e_z*Ts;end
e_th=atan2(sin(theta_d-theta),cos(theta_d-theta));
dth=(e_th-cs.theta.e_prev)/Ts;
out_th_r=cs.theta.Kp*e_th+cs.theta.Ki*cs.theta.integral+cs.theta.Kd*dth;
out_th=max(-cs.theta.sat,min(cs.theta.sat,out_th_r));
ff_p=(W*zg-B*zb)*sin(theta)+0.3*m55*q-m35*u*w;tau_M=m55*out_th+ff_p;
if abs(out_th_r)<=cs.theta.sat,cs.theta.integral=cs.theta.integral+e_th*Ts;end;cs.theta.e_prev=e_th;
e_chi=atan2(sin(guid.chi_d-chi_v),cos(guid.chi_d-chi_v));
dpsi=(e_chi-cs.psi.e_prev)/Ts;
out_p_r=cs.psi.Kp*e_chi+cs.psi.Ki*cs.psi.integral+cs.psi.Kd*dpsi;
out_p=max(-cs.psi.sat,min(cs.psi.sat,out_p_r));
ff_y=0.1*m66*r+m26*u*v;tau_N=m66*out_p+ff_y;
if abs(out_p_r)<=cs.psi.sat,cs.psi.integral=cs.psi.integral+e_chi*Ts;end;cs.psi.e_prev=e_chi;
tc=zeros(6,1);tc(5)=tau_M;tc(6)=tau_N;
dbg.e_u=e_u;dbg.e_chi=e_chi;dbg.e_theta=e_th;
dbg.tau_M=tau_M;dbg.tau_N=tau_N;dbg.n_direct=nd;dbg.ff_pitch=ff_p;dbg.ff_yaw=ff_y;
end

function [ui,flags]=actuation_local(tau_ctrl,n_direct,U,auv)
rho=auv.phys.rho;U_e=max(U,0.3);
dr=tau_ctrl(6)/(0.5*rho*U_e^2*auv.act.A_r*auv.act.CL_delta_r);
ds=tau_ctrl(5)/(0.5*rho*U_e^2*auv.act.A_s*auv.act.CL_delta_s);
dr=max(auv.act.delta_min,min(auv.act.delta_max,dr));
ds=max(auv.act.delta_min,min(auv.act.delta_max,ds));
n=max(auv.act.n_min,min(auv.act.n_max,n_direct));
ui=[dr;ds;n];flags=uint8([0;0;0]);
end

function pwm=ui_to_pwm_local(ui,auv)
pwm=zeros(3,1);
pwm(1)=auv.act.pwm_fin_neutral+(ui(1)/auv.act.delta_max)*auv.act.pwm_fin_range;
pwm(2)=auv.act.pwm_fin_neutral+(ui(2)/auv.act.delta_max)*auv.act.pwm_fin_range;
pwm(3)=auv.act.pwm_esc_min+(ui(3)/auv.act.n_max)*(auv.act.pwm_esc_max-auv.act.pwm_esc_min);
pwm=max(1000,min(2000,pwm));
end

% =========================================================================
% Logger & Visualization Local Copies (Phase 8)
% =========================================================================

% --- Logger Functions ---
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

% --- Viz Style Helper ---
function s = get_style_local()
% Shared style constants — keep in sync with viz_lib.m get_style()
s.lw_act=1.5; s.lw_ref=1.2; s.lw_lim=0.8; s.lw_zero=0.5;
s.fs_sgt=12;  s.fs_ttl=10;  s.fs_lbl=9;   s.fs_lgn=8;
s.c={[0.13 0.47 0.71],[0.90 0.45 0.10],[0.17 0.63 0.17], ...
     [0.84 0.15 0.16],[0.58 0.40 0.74],[0.55 0.34 0.29]};
end

% --- Viz Functions (kept local for MATLAB scoping; synced with viz_lib.m) ---
function viz = viz_init(update_every)
if nargin < 1, update_every = 50; end
viz.update_every = update_every; viz.initialized = true;
s = get_style_local();

% Figure 10 — 6-DOF Velocities
fig1 = figure(10);
set(fig1,'Name','Live — 6-DOF Velocities','NumberTitle','off','Position',[10 560 640 420]);
clf(fig1);
subplot(2,3,1);
viz.h_ud = animatedline('Color','k','LineStyle','--','LineWidth',s.lw_ref,'DisplayName','u_d');
viz.h_u  = animatedline('Color',s.c{1},'LineWidth',s.lw_act,'DisplayName','u');
title('Surge u','FontSize',s.fs_ttl); ylabel('u (m/s)','FontSize',s.fs_lbl);
legend('FontSize',s.fs_lgn,'Location','best'); grid on;
subplot(2,3,2);
viz.h_v = animatedline('Color',s.c{2},'LineWidth',s.lw_act);
yline(0,'--k','LineWidth',s.lw_ref,'Label','ref=0','HandleVisibility','off');
title('Sway v','FontSize',s.fs_ttl); ylabel('v (m/s)','FontSize',s.fs_lbl); grid on;
subplot(2,3,3);
viz.h_w = animatedline('Color',s.c{3},'LineWidth',s.lw_act);
yline(0,'--k','LineWidth',s.lw_ref,'Label','ref=0','HandleVisibility','off');
title('Heave w','FontSize',s.fs_ttl); ylabel('w (m/s)','FontSize',s.fs_lbl); grid on;
subplot(2,3,4);
viz.h_p = animatedline('Color',s.c{4},'LineWidth',s.lw_act);
yline(0,'--k','LineWidth',s.lw_ref,'Label','ref=0','HandleVisibility','off');
title('Roll Rate p','FontSize',s.fs_ttl);
xlabel('t (s)','FontSize',s.fs_lbl); ylabel('p (deg/s)','FontSize',s.fs_lbl); grid on;
subplot(2,3,5);
viz.h_q = animatedline('Color',s.c{5},'LineWidth',s.lw_act);
title('Pitch Rate q','FontSize',s.fs_ttl);
xlabel('t (s)','FontSize',s.fs_lbl); ylabel('q (deg/s)','FontSize',s.fs_lbl); grid on;
subplot(2,3,6);
viz.h_r = animatedline('Color',s.c{6},'LineWidth',s.lw_act);
title('Yaw Rate r','FontSize',s.fs_ttl);
xlabel('t (s)','FontSize',s.fs_lbl); ylabel('r (deg/s)','FontSize',s.fs_lbl); grid on;
sgtitle('Live: 6-DOF Velocities','FontSize',s.fs_sgt);
viz.fig1 = fig1;

% Figure 11 — Position & Attitude
fig2 = figure(11);
set(fig2,'Name','Live — Position & Attitude','NumberTitle','off','Position',[660 560 640 420]);
clf(fig2);
subplot(2,3,1);
viz.h_xn = animatedline('Color',s.c{1},'LineWidth',s.lw_act);
title('North x_N','FontSize',s.fs_ttl); ylabel('x_N (m)','FontSize',s.fs_lbl); grid on;
subplot(2,3,2);
viz.h_ye = animatedline('Color',s.c{2},'LineWidth',s.lw_act);
title('East y_E','FontSize',s.fs_ttl); ylabel('y_E (m)','FontSize',s.fs_lbl); grid on;
subplot(2,3,3);
viz.h_zref = animatedline('Color','k','LineStyle','--','LineWidth',s.lw_ref,'DisplayName','z_{ref}');
viz.h_zd   = animatedline('Color',s.c{3},'LineWidth',s.lw_act,'DisplayName','z_D');
title('Depth z_D','FontSize',s.fs_ttl); ylabel('z_D (m)','FontSize',s.fs_lbl);
legend('FontSize',s.fs_lgn,'Location','best'); grid on;
subplot(2,3,4);
viz.h_phi = animatedline('Color',s.c{4},'LineWidth',s.lw_act);
yline(0,'--k','LineWidth',s.lw_ref,'Label','ref=0','HandleVisibility','off');
title('Roll \phi','FontSize',s.fs_ttl);
xlabel('t (s)','FontSize',s.fs_lbl); ylabel('\phi (deg)','FontSize',s.fs_lbl); grid on;
subplot(2,3,5);
viz.h_ups   = animatedline('Color','k','LineStyle','--','LineWidth',s.lw_ref,'DisplayName','\upsilon_d');
viz.h_theta = animatedline('Color',s.c{5},'LineWidth',s.lw_act,'DisplayName','\theta');
title('Pitch \theta','FontSize',s.fs_ttl);
xlabel('t (s)','FontSize',s.fs_lbl); ylabel('\theta (deg)','FontSize',s.fs_lbl);
legend('FontSize',s.fs_lgn,'Location','best'); grid on;
subplot(2,3,6);
viz.h_chid = animatedline('Color','k','LineStyle','--','LineWidth',s.lw_ref,'DisplayName','\chi_d');
viz.h_psi  = animatedline('Color',s.c{6},'LineWidth',s.lw_act,'DisplayName','\psi');
title('Yaw \psi','FontSize',s.fs_ttl);
xlabel('t (s)','FontSize',s.fs_lbl); ylabel('\psi (deg)','FontSize',s.fs_lbl);
legend('FontSize',s.fs_lgn,'Location','best'); grid on;
sgtitle('Live: Position & Attitude','FontSize',s.fs_sgt);
viz.fig2 = fig2;

% Figure 12 — Actuator Commands
fig3 = figure(12);
set(fig3,'Name','Live — Actuator Commands','NumberTitle','off','Position',[10 80 640 420]);
clf(fig3);
subplot(3,1,1);
viz.h_n_rpm = animatedline('Color',s.c{1},'LineWidth',s.lw_act);
yline(1525,'--r','LineWidth',s.lw_lim,'Label','Max 1525 RPM','HandleVisibility','off');
yline(0,'--k','LineWidth',s.lw_zero,'HandleVisibility','off');
title('Propeller Speed','FontSize',s.fs_ttl); ylabel('n (RPM)','FontSize',s.fs_lbl);
ylim([-50 1600]); grid on;
subplot(3,1,2);
viz.h_ds = animatedline('Color',s.c{2},'LineWidth',s.lw_act);
yline( 20,'--r','LineWidth',s.lw_lim,'Label','+20°','HandleVisibility','off');
yline(-20,'--r','LineWidth',s.lw_lim,'Label','-20°','HandleVisibility','off');
title('Stern Plane \delta_s','FontSize',s.fs_ttl); ylabel('\delta_s (deg)','FontSize',s.fs_lbl);
ylim([-25 25]); grid on;
subplot(3,1,3);
viz.h_dr = animatedline('Color',s.c{3},'LineWidth',s.lw_act);
yline( 20,'--r','LineWidth',s.lw_lim,'Label','+20°','HandleVisibility','off');
yline(-20,'--r','LineWidth',s.lw_lim,'Label','-20°','HandleVisibility','off');
title('Rudder \delta_r','FontSize',s.fs_ttl);
xlabel('t (s)','FontSize',s.fs_lbl); ylabel('\delta_r (deg)','FontSize',s.fs_lbl);
ylim([-25 25]); grid on;
sgtitle('Live: Actuator Commands','FontSize',s.fs_sgt);
viz.fig3 = fig3;

% Figure 13 — LOS Cross-track Errors
fig4 = figure(13);
set(fig4,'Name','Live — LOS Cross-track Errors','NumberTitle','off','Position',[660 80 640 420]);
clf(fig4);
subplot(3,1,1);
viz.h_xe = animatedline('Color',s.c{1},'LineWidth',s.lw_act);
yline(0,'-k','LineWidth',s.lw_zero,'HandleVisibility','off');
title('Along-track Error x_e','FontSize',s.fs_ttl); ylabel('x_e (m)','FontSize',s.fs_lbl); grid on;
subplot(3,1,2);
viz.h_lye = animatedline('Color',s.c{4},'LineWidth',s.lw_act);
yline(0,'-k','LineWidth',s.lw_zero,'HandleVisibility','off');
title('Lateral Cross-track y_e','FontSize',s.fs_ttl); ylabel('y_e (m)','FontSize',s.fs_lbl); grid on;
subplot(3,1,3);
viz.h_lze = animatedline('Color',s.c{3},'LineWidth',s.lw_act);
yline(0,'-k','LineWidth',s.lw_zero,'HandleVisibility','off');
title('Vertical Cross-track z_e','FontSize',s.fs_ttl);
xlabel('t (s)','FontSize',s.fs_lbl); ylabel('z_e (m)','FontSize',s.fs_lbl); grid on;
sgtitle('Live: LOS Cross-track Errors','FontSize',s.fs_sgt);
viz.fig4 = fig4;

drawnow;
end

function viz = viz_update(viz, log)
k = log.k; if k < 1, return; end; t = log.t(k);
% Figure 10 — Velocities
addpoints(viz.h_ud, t, log.ud(k));
addpoints(viz.h_u,  t, log.x(1,k));
addpoints(viz.h_v,  t, log.x(2,k));
addpoints(viz.h_w,  t, log.x(3,k));
addpoints(viz.h_p,  t, rad2deg(log.x(4,k)));
addpoints(viz.h_q,  t, rad2deg(log.x(5,k)));
addpoints(viz.h_r,  t, rad2deg(log.x(6,k)));
% Figure 11 — Position / Attitude
addpoints(viz.h_xn,    t, log.x(7,k));
addpoints(viz.h_ye,    t, log.x(8,k));
addpoints(viz.h_zd,    t, log.x(9,k));
addpoints(viz.h_zref,  t, log.x(9,k) - log.los_ze(k));   % z_ref from LOS
addpoints(viz.h_phi,   t, rad2deg(log.x(10,k)));
addpoints(viz.h_ups,   t, rad2deg(log.upsilon_d(k)));
addpoints(viz.h_theta, t, rad2deg(log.x(11,k)));
addpoints(viz.h_chid,  t, rad2deg(log.chi_d(k)));
addpoints(viz.h_psi,   t, rad2deg(log.x(12,k)));
% Figure 12 — Actuators
addpoints(viz.h_n_rpm, t, log.n_direct(k));
addpoints(viz.h_ds,    t, rad2deg(log.ui(2,k)));
addpoints(viz.h_dr,    t, rad2deg(log.ui(1,k)));
% Figure 13 — Errors
addpoints(viz.h_xe,  t, log.los_xe(k));
addpoints(viz.h_lye, t, log.los_ye(k));
addpoints(viz.h_lze, t, log.los_ze(k));
drawnow limitrate;
end

function viz_close(viz)
for fnum = [10 11 12 13]
    if ishandle(fnum), close(fnum); end
end
fprintf('  Live figures (10-13) closed.\n');
end

function viz_final(log)
if ~isfield(log,'k') || log.k < 2, return; end
s  = get_style_local();
k  = log.k;  t = log.t(1:k);
z_ref = log.x(9,1:k) - log.los_ze(1:k);   % guidance depth reference

% --- Figure 20: 3D Trajectory ---
fig20 = figure(20);
set(fig20,'Name','Post-run — 3D Trajectory','NumberTitle','off','Position',[50 430 720 520]);
clf(fig20);
plot3(log.x(7,1:k), log.x(8,1:k), -log.x(9,1:k), ...
    'Color',s.c{1},'LineWidth',s.lw_act,'DisplayName','Vehicle path');
hold on;
scatter3(log.x(7,1), log.x(8,1), -log.x(9,1), 100,'g','filled','DisplayName','Start');
scatter3(log.x(7,k), log.x(8,k), -log.x(9,k), 100,'r','filled','DisplayName','End');
hold off;
xlabel('North (m)','FontSize',s.fs_lbl); ylabel('East (m)','FontSize',s.fs_lbl);
zlabel('Altitude — up+ve (m)','FontSize',s.fs_lbl);
title(sprintf('3D AUV Trajectory  [T = %.0f s,  d = %.0f m]', ...
    t(k), norm(log.x(7:9,k)-log.x(7:9,1))),'FontSize',s.fs_ttl);
legend('FontSize',s.fs_lgn,'Location','best');
grid on; daspect([1 1 0.05]); view(30,25);

% --- Figure 21: All 12 States ---
fig21 = figure(21);
set(fig21,'Name','Post-run — All 12 States','NumberTitle','off','Position',[50 30 1140 720]);
clf(fig21);
vel_titles  = {'Surge u','Sway v','Heave w','Roll Rate p','Pitch Rate q','Yaw Rate r'};
vel_ylabels = {'u (m/s)','v (m/s)','w (m/s)','p (deg/s)','q (deg/s)','r (deg/s)'};
for i = 1:6
    ax = subplot(4,3,i); hold(ax,'on');
    switch i
        case 1
            h1 = plot(t, log.ud(1:k),'--k','LineWidth',s.lw_ref,'DisplayName','u_d');
            h2 = plot(t, log.x(1,1:k),'Color',s.c{i},'LineWidth',s.lw_act,'DisplayName','u');
            legend([h1 h2],'FontSize',s.fs_lgn,'Location','best');
        case {2,3}
            yline(0,'--k','LineWidth',s.lw_ref,'Label','ref=0','HandleVisibility','off');
            plot(t, log.x(i,1:k),'Color',s.c{i},'LineWidth',s.lw_act);
        case 4
            yline(0,'--k','LineWidth',s.lw_ref,'Label','ref=0','HandleVisibility','off');
            plot(t, rad2deg(log.x(i,1:k)),'Color',s.c{i},'LineWidth',s.lw_act);
        case {5,6}
            plot(t, rad2deg(log.x(i,1:k)),'Color',s.c{i},'LineWidth',s.lw_act);
    end
    title(vel_titles{i},'FontSize',s.fs_ttl);
    xlabel('t (s)','FontSize',s.fs_lbl); ylabel(vel_ylabels{i},'FontSize',s.fs_lbl);
    grid on; hold(ax,'off');
end
pos_titles  = {'North x_N','East y_E','Depth z_D','Roll \phi','Pitch \theta','Yaw \psi'};
pos_ylabels = {'x_N (m)','y_E (m)','z_D (m)','\phi (deg)','\theta (deg)','\psi (deg)'};
for i = 1:6
    ax = subplot(4,3,6+i); hold(ax,'on');
    switch i
        case {1,2}
            plot(t, log.x(6+i,1:k),'Color',s.c{i},'LineWidth',s.lw_act);
        case 3
            h1 = plot(t, z_ref,'--k','LineWidth',s.lw_ref,'DisplayName','z_{ref}');
            h2 = plot(t, log.x(9,1:k),'Color',s.c{i},'LineWidth',s.lw_act,'DisplayName','z_D');
            legend([h1 h2],'FontSize',s.fs_lgn,'Location','best');
        case 4
            yline(0,'--k','LineWidth',s.lw_ref,'Label','ref=0','HandleVisibility','off');
            plot(t, rad2deg(log.x(10,1:k)),'Color',s.c{i},'LineWidth',s.lw_act);
        case 5
            h1 = plot(t, rad2deg(log.upsilon_d(1:k)),'--k','LineWidth',s.lw_ref,'DisplayName','\upsilon_d');
            h2 = plot(t, rad2deg(log.x(11,1:k)),'Color',s.c{i},'LineWidth',s.lw_act,'DisplayName','\theta');
            legend([h1 h2],'FontSize',s.fs_lgn,'Location','best');
        case 6
            h1 = plot(t, rad2deg(log.chi_d(1:k)),'--k','LineWidth',s.lw_ref,'DisplayName','\chi_d');
            h2 = plot(t, rad2deg(log.x(12,1:k)),'Color',s.c{i},'LineWidth',s.lw_act,'DisplayName','\psi');
            legend([h1 h2],'FontSize',s.fs_lgn,'Location','best');
    end
    title(pos_titles{i},'FontSize',s.fs_ttl);
    xlabel('t (s)','FontSize',s.fs_lbl); ylabel(pos_ylabels{i},'FontSize',s.fs_lbl);
    grid on; hold(ax,'off');
end
sgtitle('Post-run: All 12 States — Desired vs Actual','FontSize',s.fs_sgt);

% --- Figure 22: Actuator Signals ---
fig22 = figure(22);
set(fig22,'Name','Post-run — Actuator Signals','NumberTitle','off','Position',[800 430 640 460]);
clf(fig22);
subplot(3,1,1);
plot(t, log.n_direct(1:k),'Color',s.c{1},'LineWidth',s.lw_act);
hold on;
yline(1525,'--r','LineWidth',s.lw_lim,'Label','Max 1525 RPM','HandleVisibility','off');
yline(0,'--k','LineWidth',s.lw_zero,'HandleVisibility','off');
hold off;
title('Propeller Speed','FontSize',s.fs_ttl);
ylabel('n (RPM)','FontSize',s.fs_lbl); xlabel('t (s)','FontSize',s.fs_lbl);
ylim([-50 1600]); grid on;
subplot(3,1,2);
plot(t, rad2deg(log.ui(2,1:k)),'Color',s.c{2},'LineWidth',s.lw_act);
hold on;
yline( 20,'--r','LineWidth',s.lw_lim,'Label','+20° limit','HandleVisibility','off');
yline(-20,'--r','LineWidth',s.lw_lim,'Label','-20° limit','HandleVisibility','off');
hold off;
title('Stern Plane \delta_s','FontSize',s.fs_ttl);
ylabel('\delta_s (deg)','FontSize',s.fs_lbl); xlabel('t (s)','FontSize',s.fs_lbl);
ylim([-25 25]); grid on;
subplot(3,1,3);
plot(t, rad2deg(log.ui(1,1:k)),'Color',s.c{3},'LineWidth',s.lw_act);
hold on;
yline( 20,'--r','LineWidth',s.lw_lim,'Label','+20° limit','HandleVisibility','off');
yline(-20,'--r','LineWidth',s.lw_lim,'Label','-20° limit','HandleVisibility','off');
hold off;
title('Rudder \delta_r','FontSize',s.fs_ttl);
ylabel('\delta_r (deg)','FontSize',s.fs_lbl); xlabel('t (s)','FontSize',s.fs_lbl);
ylim([-25 25]); grid on;
sgtitle('Post-run: Actuator Signals','FontSize',s.fs_sgt);

% --- Figure 23: LOS Cross-track Errors ---
fig23 = figure(23);
set(fig23,'Name','Post-run — LOS Cross-track Errors','NumberTitle','off','Position',[800 30 640 440]);
clf(fig23);
rms_xe = sqrt(mean(log.los_xe(1:k).^2));
rms_ye = sqrt(mean(log.los_ye(1:k).^2));
rms_ze = sqrt(mean(log.los_ze(1:k).^2));
subplot(3,1,1);
plot(t, log.los_xe(1:k),'Color',s.c{1},'LineWidth',s.lw_act);
yline(0,'-k','LineWidth',s.lw_zero,'HandleVisibility','off');
title(sprintf('Along-track Error x_e    [RMS = %.2f m]', rms_xe),'FontSize',s.fs_ttl);
ylabel('x_e (m)','FontSize',s.fs_lbl); xlabel('t (s)','FontSize',s.fs_lbl); grid on;
subplot(3,1,2);
plot(t, log.los_ye(1:k),'Color',s.c{4},'LineWidth',s.lw_act);
yline(0,'-k','LineWidth',s.lw_zero,'HandleVisibility','off');
title(sprintf('Lateral Cross-track y_e    [RMS = %.2f m]', rms_ye),'FontSize',s.fs_ttl);
ylabel('y_e (m)','FontSize',s.fs_lbl); xlabel('t (s)','FontSize',s.fs_lbl); grid on;
subplot(3,1,3);
plot(t, log.los_ze(1:k),'Color',s.c{3},'LineWidth',s.lw_act);
yline(0,'-k','LineWidth',s.lw_zero,'HandleVisibility','off');
title(sprintf('Vertical Cross-track z_e    [RMS = %.2f m]', rms_ze),'FontSize',s.fs_ttl);
ylabel('z_e (m)','FontSize',s.fs_lbl); xlabel('t (s)','FontSize',s.fs_lbl); grid on;
sgtitle('Post-run: LOS Cross-track Errors','FontSize',s.fs_sgt);

% --- Figure 24: Environmental Disturbances ---
fig24 = figure(24);
set(fig24,'Name','Post-run — Environmental Disturbances','NumberTitle','off','Position',[50 30 700 420]);
clf(fig24);
subplot(2,2,1);
plot(t, log.Vc(1:k),'Color',s.c{1},'LineWidth',s.lw_act);
title('Ocean Current Speed','FontSize',s.fs_ttl);
xlabel('t (s)','FontSize',s.fs_lbl); ylabel('V_c (m/s)','FontSize',s.fs_lbl); grid on;
subplot(2,2,2);
plot(t, rad2deg(log.betaVc(1:k)),'Color',s.c{2},'LineWidth',s.lw_act);
title('Current Direction','FontSize',s.fs_ttl);
xlabel('t (s)','FontSize',s.fs_lbl); ylabel('\beta_{Vc} (deg)','FontSize',s.fs_lbl); grid on;
subplot(2,2,3);
plot(t, log.tau_env(3,1:k),'Color',s.c{3},'LineWidth',s.lw_act);
yline(0,'-k','LineWidth',s.lw_zero,'HandleVisibility','off');
title('Wave Heave Force \tau_Z','FontSize',s.fs_ttl);
xlabel('t (s)','FontSize',s.fs_lbl); ylabel('\tau_Z (N)','FontSize',s.fs_lbl); grid on;
subplot(2,2,4);
plot(t, log.tau_env(5,1:k),'Color',s.c{4},'LineWidth',s.lw_act);
yline(0,'-k','LineWidth',s.lw_zero,'HandleVisibility','off');
title('Wave Pitch Moment \tau_M','FontSize',s.fs_ttl);
xlabel('t (s)','FontSize',s.fs_lbl); ylabel('\tau_M (N·m)','FontSize',s.fs_lbl); grid on;
sgtitle('Post-run: Environmental Disturbances','FontSize',s.fs_sgt);

fprintf('\n  Post-run figures 20-24 generated.\n');
fprintf('  RMS cross-track:  x_e=%.3fm  y_e=%.3fm  z_e=%.3fm\n', rms_xe, rms_ye, rms_ze);
end