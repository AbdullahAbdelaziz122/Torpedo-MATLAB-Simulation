%% run_simulation_hil.m  —  AUV Simulation Runner with HIL Support  (Phase 10)
%
% PURPOSE:
%   Extends run_simulation.m with the HIL communication layer (M7).
%   The HIL layer is entirely isolated — all simulation physics, control,
%   guidance, and logging are identical to the SIL version.
%
% MODES:
%   'sil'   — Software-in-the-loop (no hardware, default)
%   'hil'   — Hardware-in-the-loop (ESP32 connected via UART)
%   'fast'  — SIL, no plots, maximum throughput
%
% HIL OPERATION:
%   At each HIL step (every Ts_HIL = 0.02s = 50 Hz):
%     1. Simulation computes PWM values from Actuation module
%     2. hil_send() transmits 9-byte frame to ESP32
%     3. hil_receive() reads optional 10-byte telemetry echo
%     4. Simulation continues — ESP32 drives real actuators
%
%   The simulation does NOT wait for ESP32 acknowledgement.
%   This is a one-way command stream — the ESP32 watchdog handles
%   communication loss by reverting to safe state after 500ms.
%
% USAGE:
%   >> buses; auv_params; auv_params_env_patch;
%   >> run_simulation_hil('sil')              % SIL, live plots
%   >> run_simulation_hil('hil', 'COM3')      % HIL on Windows
%   >> run_simulation_hil('hil', '/dev/ttyUSB0')  % HIL on Linux
%   >> run_simulation_hil('fast')             % SIL, no plots
%
% AUTHOR: AUV Simulation Project — Phase 10

function log = run_simulation_hil(mode, hil_port)

if nargin < 1, mode     = 'sil';  end
if nargin < 2, hil_port = 'COM3'; end

hil_enabled = strcmp(mode, 'hil');
live_plots  = strcmp(mode, 'sil');
show_final  = ~strcmp(mode, 'fast');

% =========================================================================
% Setup
% =========================================================================
if ~exist('auv','var')
    error('Run ''buses; auv_params; auv_params_env_patch'' first.');
end
auv_sim = evalin('base', 'auv');

fprintf('\n=== AUV Simulation (Phase 10) ===\n');
fprintf('  Mode:        %s\n', mode);
fprintf('  HIL enabled: %d\n', hil_enabled);
if hil_enabled
    fprintf('  HIL port:    %s\n', hil_port);
end
fprintf('  Duration:    %.0f s\n', auv_sim.sim.T_end);

dt      = auv_sim.sim.Ts;
Ts_HIL  = auv_sim.sim.Ts_HIL;   % 0.02s = 50 Hz HIL rate
T_end   = auv_sim.sim.T_end;
N       = round(T_end / dt) + 1;
tvec    = 0 : dt : T_end;

% HIL step ratio: how many sim steps per HIL transmission
hil_ratio = round(Ts_HIL / dt);   % = 2 at Ts=0.01, Ts_HIL=0.02

% =========================================================================
% Initialise all modules
% =========================================================================
env_state = env_init_local(auv_sim);
path      = path_helix_local(auv_sim);
los       = los_init_local(auv_sim);
cs        = ctrl_init_local(auv_sim);
log       = log_init_local(N);

if live_plots
    viz = viz_init_local();
end

% M7 — HIL initialisation (isolated, no effect on other modules)
hil = hil_init_local(hil_port, 115200, hil_enabled);

% Initial conditions
x0       = zeros(12, 1);
x0(1)    = 1.5;
x0(7)    = path.pts(1,1);
x0(8)    = path.pts(2,1);
x0(9)    = path.pts(3,1);
x0(12)   = path.psi0;
x        = x0;

% =========================================================================
% Simulation loop
% =========================================================================
fprintf('\n  Running...\n');
tic;

for k = 1:N
    t = tvec(k);

    % --- M6: Navigation ---
    nu_hat  = x(1:6);
    eta_hat = x(7:12);
    eta_hat(4:6) = arrayfun(@(a) atan2(sin(a),cos(a)), eta_hat(4:6));

    % --- M1: Environment ---
    [Vc, betaVc, w_c, tau_env, env_state] = env_step_local(env_state, t);

    % --- M2: Guidance ---
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

    % --- M3: Control ---
    [tau_ctrl, n_direct, ctrl_dbg, cs] = ctrl_step_local(guid, nu_hat, eta_hat, cs);

    % --- M4: Actuation ---
    [ui, sat_flags] = actuation_local(tau_ctrl, n_direct, nu_hat(1), auv_sim);
    pwm             = ui_to_pwm_local(ui, auv_sim);

    % -----------------------------------------------------------------------
    % M7: HIL transmission (every hil_ratio steps = 50 Hz)
    % Isolated block — removing this section leaves all other logic intact
    % -----------------------------------------------------------------------
    if hil_enabled && mod(k-1, hil_ratio) == 0
        hil = hil_send_local(hil, pwm);
        [hil, ~, ~] = hil_receive_local(hil);
    end
    % -----------------------------------------------------------------------

    % --- M7: Log ---
    nav_log.nu_hat    = nu_hat;
    nav_log.eta_hat   = eta_hat;
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
    log = log_step_local(log, t, x, nav_log, ctrl_log, act_log, env_log, guid);

    % --- M7: Live viz ---
    if live_plots && mod(k-1, 50) == 0
        viz = viz_update_local(viz, log);
    end

    % --- M5: Dynamics (RK4) ---
    if k < N
        x = rk4_step_local(x, ui, Vc, betaVc, w_c, tau_env, dt);
    end

    % Progress report every 10s
    if mod(k-1, round(10/dt)) == 0 && k > 1
        elapsed = toc;
        fprintf('  t=%5.1fs  u=%.2fm/s  z=%.2fm  psi=%.1fdeg  wall=%.1fs\n', ...
            t, x(1), x(9), rad2deg(x(12)), elapsed);
    end
end

sim_time = toc;

% =========================================================================
% Finalise
% =========================================================================
% Safe shutdown of HIL
if hil_enabled
    safe_pwm = [1500; 1500; 1000];
    hil_send_local(hil, safe_pwm);
    pause(0.1);
    hil_close_local(hil);
end

log = log_trim_local(log);
log_summary_local(log);

filename = log_save_local(log, sprintf('auv_%s_log', mode));
fprintf('  Wall time: %.1fs  (%.1fx real-time)\n\n', sim_time, T_end/sim_time);

if show_final
    viz_final_local(log);
end

assignin('base', 'log', log);

end

% =========================================================================
% RK4 integration
% =========================================================================
function x_next = rk4_step_local(x, ui, Vc, betaVc, w_c, tau_env, dt)
[~,~,M] = remus100();
a_env   = [M \ tau_env(1:6); zeros(6,1)];
f = @(xx) remus100(xx, ui, Vc, betaVc, w_c) + a_env;
k1 = f(x);
k2 = f(x + (dt/2)*k1);
k3 = f(x + (dt/2)*k2);
k4 = f(x + dt*k3);
x_next = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end

% =========================================================================
% HIL local functions (isolated — only called when hil_enabled = true)
% =========================================================================
function hil = hil_init_local(port, baud, enabled)
hil.enabled=enabled; hil.port=port; hil.baud=baud;
hil.tx_count=0; hil.rx_count=0; hil.err_count=0;
hil.last_status=0; hil.last_echo=[1500;1500;1000]; hil.armed=false;
if ~enabled, hil.serial=[]; return, end
try
    s=serialport(port,baud); s.Timeout=0.01; flush(s);
    hil.serial=s;
    fprintf('HIL: opened %s at %d baud.\n',port,baud);
catch ME
    warning('HIL: failed to open %s — %s\nFalling back to SIL.',port,ME.message);
    hil.serial=[]; hil.enabled=false;
end
end

function hil = hil_send_local(hil, pwm)
if ~hil.enabled, return, end
pwm = max(1000,min(2000,double(pwm(:))));
p1=uint16(round(pwm(1))); p2=uint16(round(pwm(2))); p3=uint16(round(pwm(3)));
frame=uint8([0xAB;0xCD;bitshift(p1,-8);bitand(p1,0xFF);...
             bitshift(p2,-8);bitand(p2,0xFF);bitshift(p3,-8);bitand(p3,0xFF);0x00]);
frame(9)=hil_crc8_local(frame(1:8));
try
    write(hil.serial,frame,'uint8'); hil.tx_count=hil.tx_count+1;
catch, hil.err_count=hil.err_count+1; end
end

function [hil,status,echo_pwm] = hil_receive_local(hil)
status=hil.last_status; echo_pwm=hil.last_echo;
if ~hil.enabled||isempty(hil.serial), return, end
try
    if hil.serial.NumBytesAvailable < 10, return, end
    raw=read(hil.serial,10,'uint8');
    if raw(1)~=0xDC||raw(2)~=0xBA, hil.err_count=hil.err_count+1; return, end
    if hil_crc8_local(raw(1:9))~=raw(10), hil.err_count=hil.err_count+1; return, end
    status=raw(3);
    echo_pwm=[double(bitor(bitshift(uint16(raw(4)),8),uint16(raw(5))));
              double(bitor(bitshift(uint16(raw(6)),8),uint16(raw(7))));
              double(bitor(bitshift(uint16(raw(8)),8),uint16(raw(9))))];
    hil.last_status=status; hil.last_echo=echo_pwm;
    hil.armed=bitand(status,1)>0; hil.rx_count=hil.rx_count+1;
catch, end
end

function hil_close_local(hil)
if ~hil.enabled||isempty(hil.serial), return, end
flush(hil.serial); delete(hil.serial);
fprintf('HIL closed. TX=%d RX=%d Errors=%d\n',...
    hil.tx_count,hil.rx_count,hil.err_count);
end

function crc = hil_crc8_local(data)
crc=uint8(0); poly=uint8(0x07);
for i=1:numel(data)
    crc=bitxor(crc,uint8(data(i)));
    for b=1:8
        if bitand(crc,uint8(0x80))>0, crc=bitxor(bitshift(crc,1),poly);
        else, crc=bitshift(crc,1); end
    end
end
end

% =========================================================================
% All other module local copies (identical to run_simulation.m)
% =========================================================================
function es=env_init_local(auv)
if ~isfield(auv,'env'),es=env_zero_local();return,end
e=auv.env;es.Vc=e.Vc_mean;es.betaVc=e.betaVc_mean;es.w_c=e.w_c;
es.Vc_gm=e.Vc_mean;es.beta_gm=e.betaVc_mean;es.wave_x=zeros(6,1);
es.wave_on=e.wave_on;es.dt=auv.sim.Ts;
es.mu_Vc=e.mu_Vc;es.mu_beta=e.mu_betaVc;
es.sigma_Vc=e.sigma_Vc;es.sigma_beta=e.sigma_betaVc;
if e.wave_on&&e.Hs>0
    es.wf_Z=wf_i(e.Hs,e.Tp,e.wave_scale_Z);
    es.wf_K=wf_i(e.Hs*0.3,e.Tp,e.wave_scale_K);
    es.wf_M=wf_i(e.Hs*0.5,e.Tp,e.wave_scale_M);
else, es.wf_Z=wf_i(0,6,0);es.wf_K=wf_i(0,6,0);es.wf_M=wf_i(0,6,0);
end
function wf=wf_i(Hs,Tp,sc)
if Hs<=0||Tp<=0,wf.omega_n=1;wf.zeta=0.1;wf.Kw=0;return,end
wf.omega_n=2*pi/Tp;wf.zeta=0.1;wf.Kw=sc*(Hs/4)*sqrt(2*wf.zeta*wf.omega_n);
function es=env_zero_local()
es.Vc=0;es.betaVc=0;es.w_c=0;es.Vc_gm=0;es.beta_gm=0;es.wave_x=zeros(6,1);
es.wave_on=false;es.dt=0.01;es.mu_Vc=0;es.mu_beta=0;es.sigma_Vc=0;es.sigma_beta=0;
es.wf_Z=wf_i(0,6,0);es.wf_K=wf_i(0,6,0);es.wf_M=wf_i(0,6,0);
function [Vc,bVc,wc,tau_env,es]=env_step_local(es,t) %#ok<INUSL>
dt=es.dt;
es.Vc_gm=gm(es.Vc_gm,es.mu_Vc,es.sigma_Vc,dt);
es.beta_gm=gm(es.beta_gm,es.mu_beta,es.sigma_beta,dt);
Vc=max(0,es.Vc_gm);bVc=atan2(sin(es.beta_gm),cos(es.beta_gm));wc=es.w_c;
es.Vc=Vc;es.betaVc=bVc;tau_env=zeros(6,1);
if es.wave_on
    [tau_env(3),es.wave_x(1:2)]=wfs(es.wave_x(1:2),es.wf_Z,dt);
    [tau_env(4),es.wave_x(3:4)]=wfs(es.wave_x(3:4),es.wf_K,dt);
    [tau_env(5),es.wave_x(5:6)]=wfs(es.wave_x(5:6),es.wf_M,dt);
end
function [o,xn]=wfs(x,wf,dt)
w=randn;dx1=x(2);dx2=-wf.omega_n^2*x(1)-2*wf.zeta*wf.omega_n*x(2)+wf.Kw*w;
xn=[x(1)+dt*dx1;x(2)+dt*dx2];o=xn(1);
function xn=gm(x,mu,sigma,dt)
if sigma<=0,xn=x;return,end;xn=x+dt*(-mu*x+sigma*randn);

function path=path_helix_local(auv)
T=auv.sim.T_end;dt=auv.sim.Ts;time=0:dt:T;m=0.6;Yr=m*time;
X=60*cos(0.02618*Yr);Y=60*sin(0.02618*Yr);Z=2+(2*Yr/200);
dX=-60*0.02618*m*sin(0.02618*Yr);dY=60*0.02618*m*cos(0.02618*Yr);
dZ=2*m/200*ones(size(time));
path.type='helix';path.t_vec=time;path.pts=[X;Y;Z];path.vel=[dX;dY;dZ];
path.T_total=T;path.U_nom=m;path.psi0=atan2(dY(1),dX(1));
path.len=sum(sqrt(diff(X).^2+diff(Y).^2+diff(Z).^2));

function [pt,vel]=path_query_local(path,t)
t=max(0,min(t,path.T_total));
idx=max(1,min(round(t/(path.t_vec(2)-path.t_vec(1)))+1,size(path.pts,2)));
pt=path.pts(:,idx);vel=path.vel(:,idx);

function los=los_init_local(auv)
los.ky=auv.guid.k_y;los.kz=auv.guid.k_z;
los.delta_y=auv.guid.delta_y;los.delta_z=auv.guid.delta_z;
los.U_max=auv.guid.U_max;los.U_min=auv.guid.U_min;los.U_nom=1.5;

function [cd,uda,uds,cv,uv,errs]=los_step_local(nu,eta,pt,pv,ds,los)
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
uda=asin(arg);
cy=cos(pp)*sin(pr)*cos(tr)-sin(tp)*sin(tr)*sin(pp)+sin(pp)*cos(pr)*cos(tp)*cos(tr);
cx=-sin(pp)*sin(pr)*cos(tr)-sin(tp)*sin(tr)*cos(pp)+cos(pp)*cos(pr)*cos(tp)*cos(tr);
cd=atan2(cy,cx);
denom=max(cos(pr)*cos(tr),0.1);raw=(norm(pv)-0.001*xe)/denom;
u=ds(1);v=ds(2);w=ds(3);mag=sqrt(u^2+v^2+w^2);
if mag<1e-6,uds=los.U_nom;else,uds=raw*u/mag;end
uds=max(los.U_min,min(los.U_max,uds));errs.x_e=xe;errs.y_e=ye;errs.z_e=ze;

function cs=ctrl_init_local(auv)
cs.dt=auv.sim.Ts;
cs.u.integral=0;cs.u.e_prev=0;cs.u.Kp=auv.ctrl.u.Kp;
cs.u.Ki=auv.ctrl.u.Ki;cs.u.Kd=auv.ctrl.u.Kd;cs.u.sat=auv.ctrl.u.u_max;
cs.z.integral=0;cs.z.Kp=auv.ctrl.z.Kp;cs.z.Ki=auv.ctrl.z.Ki;
cs.z.theta_max=auv.ctrl.z.theta_d_max;
cs.theta.integral=0;cs.theta.e_prev=0;cs.theta.Kp=auv.ctrl.theta.Kp;
cs.theta.Ki=auv.ctrl.theta.Ki;cs.theta.Kd=auv.ctrl.theta.Kd;
cs.theta.sat=auv.ctrl.theta.sat;
cs.psi.integral=0;cs.psi.e_prev=0;cs.psi.Kp=auv.ctrl.psi.Kp;
cs.psi.Ki=auv.ctrl.psi.Ki;cs.psi.Kd=auv.ctrl.psi.Kd;cs.psi.sat=auv.ctrl.psi.sat;
[~,~,M]=remus100();cs.m11=M(1,1);cs.m55=M(5,5);cs.m66=M(6,6);
cs.m35=M(3,5);cs.m26=M(2,6);cs.W=auv.phys.W;cs.B=auv.phys.B;
cs.zg=auv.phys.r_bG(3);cs.zb=auv.phys.r_bB(3);

function [tc,nd,dbg,cs]=ctrl_step_local(guid,nu,eta,cs)
u=nu(1);v=nu(2);w=nu(3);q=nu(5);r=nu(6);theta=eta(5);
chi_v=atan2(sin(eta(6)),cos(eta(6)));Ts=cs.dt;
[~,~,M]=remus100();m55=M(5,5);m66=M(6,6);m35=M(3,5);m26=M(2,6);
W=cs.W;B=cs.B;zg=cs.zg;zb=cs.zb;
e_u=guid.ud-u;du=(e_u-cs.u.e_prev)/Ts;
out_u=cs.u.Kp*e_u+cs.u.Ki*cs.u.integral+cs.u.Kd*du;
nd=max(0,min(1525,(1525/20)*out_u));
if abs(out_u)<=cs.u.sat,cs.u.integral=cs.u.integral+e_u*Ts;end;cs.u.e_prev=e_u;
z_d=eta(3);e_z=guid.z_des-z_d;out_z=cs.z.Kp*e_z+cs.z.Ki*cs.z.integral;
theta_d=max(-cs.z.theta_max,min(cs.z.theta_max,out_z));
if abs(out_z)<=cs.z.theta_max,cs.z.integral=cs.z.integral+e_z*Ts;end
e_th=atan2(sin(theta_d-theta),cos(theta_d-theta));dth=(e_th-cs.theta.e_prev)/Ts;
out_th_r=cs.theta.Kp*e_th+cs.theta.Ki*cs.theta.integral+cs.theta.Kd*dth;
out_th=max(-cs.theta.sat,min(cs.theta.sat,out_th_r));
ff_p=(W*zg-B*zb)*sin(theta)+0.3*m55*q-m35*u*w;tau_M=m55*out_th+ff_p;
if abs(out_th_r)<=cs.theta.sat,cs.theta.integral=cs.theta.integral+e_th*Ts;end
cs.theta.e_prev=e_th;
e_chi=atan2(sin(guid.chi_d-chi_v),cos(guid.chi_d-chi_v));dpsi=(e_chi-cs.psi.e_prev)/Ts;
out_p_r=cs.psi.Kp*e_chi+cs.psi.Ki*cs.psi.integral+cs.psi.Kd*dpsi;
out_p=max(-cs.psi.sat,min(cs.psi.sat,out_p_r));
ff_y=0.1*m66*r+m26*u*v;tau_N=m66*out_p+ff_y;
if abs(out_p_r)<=cs.psi.sat,cs.psi.integral=cs.psi.integral+e_chi*Ts;end;cs.psi.e_prev=e_chi;
tc=zeros(6,1);tc(5)=tau_M;tc(6)=tau_N;
dbg.e_u=e_u;dbg.e_chi=e_chi;dbg.e_theta=e_th;
dbg.tau_M=tau_M;dbg.tau_N=tau_N;dbg.n_direct=nd;dbg.ff_pitch=ff_p;dbg.ff_yaw=ff_y;

function [ui,flags]=actuation_local(tau_ctrl,n_direct,U,auv)
rho=auv.phys.rho;U_e=max(U,0.3);
dr=tau_ctrl(6)/(0.5*rho*U_e^2*auv.act.A_r*auv.act.CL_delta_r);
ds=tau_ctrl(5)/(0.5*rho*U_e^2*auv.act.A_s*auv.act.CL_delta_s);
dr=max(auv.act.delta_min,min(auv.act.delta_max,dr));
ds=max(auv.act.delta_min,min(auv.act.delta_max,ds));
n=max(auv.act.n_min,min(auv.act.n_max,n_direct));
ui=[dr;ds;n];flags=uint8([0;0;0]);

function pwm=ui_to_pwm_local(ui,auv)
pwm=zeros(3,1);
pwm(1)=auv.act.pwm_fin_neutral+(ui(1)/auv.act.delta_max)*auv.act.pwm_fin_range;
pwm(2)=auv.act.pwm_fin_neutral+(ui(2)/auv.act.delta_max)*auv.act.pwm_fin_range;
pwm(3)=auv.act.pwm_esc_min+(ui(3)/auv.act.n_max)*(auv.act.pwm_esc_max-auv.act.pwm_esc_min);
pwm=max(1000,min(2000,pwm));

function log=log_init_local(N)
log.t=zeros(1,N);log.x=zeros(12,N);log.nu_hat=zeros(6,N);
log.eta_hat=zeros(6,N);log.tau_ctrl=zeros(6,N);log.ui=zeros(3,N);
log.pwm=zeros(3,N);log.sat_flags=zeros(3,N,'uint8');log.n_direct=zeros(1,N);
log.e_chi=zeros(1,N);log.e_theta=zeros(1,N);log.e_u=zeros(1,N);
log.Vc=zeros(1,N);log.betaVc=zeros(1,N);log.tau_env=zeros(6,N);
log.chi_d=zeros(1,N);log.upsilon_d=zeros(1,N);log.ud=zeros(1,N);
log.los_xe=zeros(1,N);log.los_ye=zeros(1,N);log.los_ze=zeros(1,N);
log.N=N;log.k=0;

function log=log_step_local(log,t,x,nav,ctrl,act,env,guid)
k=log.k+1;if k>log.N,return,end;log.k=k;log.t(k)=t;log.x(:,k)=x;
if isfield(nav,'nu_hat'),log.nu_hat(:,k)=nav.nu_hat;end
if isfield(nav,'eta_hat'),log.eta_hat(:,k)=nav.eta_hat;end
if isfield(ctrl,'tau_ctrl'),log.tau_ctrl(:,k)=ctrl.tau_ctrl;end
if isfield(ctrl,'n_direct'),log.n_direct(k)=ctrl.n_direct;end
if isfield(ctrl,'e_chi'),log.e_chi(k)=ctrl.e_chi;end
if isfield(ctrl,'e_theta'),log.e_theta(k)=ctrl.e_theta;end
if isfield(ctrl,'e_u'),log.e_u(k)=ctrl.e_u;end
if isfield(act,'ui'),log.ui(:,k)=act.ui;end
if isfield(act,'pwm'),log.pwm(:,k)=act.pwm;end
if isfield(act,'sat_flags'),log.sat_flags(:,k)=act.sat_flags;end
if isfield(env,'Vc'),log.Vc(k)=env.Vc;end
if isfield(env,'betaVc'),log.betaVc(k)=env.betaVc;end
if isfield(env,'tau_env'),log.tau_env(:,k)=env.tau_env;end
if isfield(guid,'chi_d'),log.chi_d(k)=guid.chi_d;end
if isfield(guid,'upsilon_d'),log.upsilon_d(k)=guid.upsilon_d;end
if isfield(guid,'ud'),log.ud(k)=guid.ud;end
if isfield(guid,'los_xe'),log.los_xe(k)=guid.los_xe;end
if isfield(guid,'los_ye'),log.los_ye(k)=guid.los_ye;end
if isfield(guid,'los_ze'),log.los_ze(k)=guid.los_ze;end

function log=log_trim_local(log)
k=log.k;f=fieldnames(log);
for i=1:numel(f)
    fn=f{i};if strcmp(fn,'N')||strcmp(fn,'k'),continue,end
    v=log.(fn);
    if isvector(v)&&numel(v)==log.N,log.(fn)=v(1:k);
    elseif ~isvector(v)&&size(v,2)==log.N,log.(fn)=v(:,1:k);end
end;log.N=k;

function log_summary_local(log)
k=log.k;if k==0,return,end
fprintf('\n=== Log Summary ===\n');
fprintf('  Duration: %.1fs (%d steps)\n',log.t(k),k);
fprintf('  u_max=%.2f  z_range=[%.1f,%.1f]m\n',...
    max(abs(log.x(1,1:k))),min(log.x(9,1:k)),max(log.x(9,1:k)));
if any(log.los_ye(1:k)~=0)
    fprintf('  RMS lateral x-track: %.2fm\n',sqrt(mean(log.los_ye(1:k).^2)));
end
if any(isnan(log.x(:,1:k)),'all'),fprintf('  WARNING: NaN in state!\n');
else,fprintf('  State integrity: OK\n');end
fprintf('===================\n\n');

function fn=log_save_local(log,prefix)
ts=datestr(now,'yyyymmdd_HHMMSS');fn=sprintf('%s_%s.mat',prefix,ts);
save(fn,'log');fprintf('  Log saved: %s\n',fn);

function viz=viz_init_local()
viz.fig=figure(10);set(viz.fig,'Name','Live — AUV HIL','NumberTitle','off');
viz.h_u  =subplot(2,2,1);viz.lu=animatedline('Color',[0.13 0.47 0.71],'LineWidth',1.2);
xlabel('t(s)');ylabel('u (m/s)');title('Surge');grid on;
viz.h_psi=subplot(2,2,2);viz.lp=animatedline('Color',[0.84 0.15 0.16],'LineWidth',1.2);
xlabel('t(s)');ylabel('ψ (deg)');title('Heading');grid on;
viz.h_z  =subplot(2,2,3);viz.lz=animatedline('Color',[0.17 0.63 0.17],'LineWidth',1.2);
xlabel('t(s)');ylabel('z (m)');title('Depth');grid on;set(gca,'YDir','reverse');
viz.h_ye =subplot(2,2,4);viz.ly=animatedline('Color',[0.90 0.45 0.10],'LineWidth',1.2);
xlabel('t(s)');ylabel('y_e (m)');title('Lateral x-track');grid on;yline(0,'-k');
drawnow;

function viz=viz_update_local(viz,log)
k=log.k;if k<1,return,end;t=log.t(k);
addpoints(viz.lu, t,log.x(1,k));
addpoints(viz.lp, t,rad2deg(log.x(12,k)));
addpoints(viz.lz, t,log.x(9,k));
addpoints(viz.ly, t,log.los_ye(k));
drawnow limitrate;

function viz_final_local(log)
if log.k<2,return,end
k=log.k;t=log.t(1:k);lw=1.5;
figure(20);set(gcf,'Name','Post-run 3D','NumberTitle','off');
plot3(log.x(7,1:k),log.x(8,1:k),-log.x(9,1:k),'LineWidth',lw);
xlabel('x_N');ylabel('y_E');zlabel('Alt');
title('3D trajectory');grid on;daspect([1 1 0.05]);view(30,25);
figure(22);set(gcf,'Name','Post-run — Actuators','NumberTitle','off');
subplot(3,1,1);plot(t,log.n_direct(1:k),'LineWidth',lw);
yline(1525,'--r');ylabel('RPM');grid on;title('Propeller');
subplot(3,1,2);plot(t,rad2deg(log.ui(2,1:k)),'LineWidth',lw);
yline(20,'--r');yline(-20,'--r');ylabel('δ_s (°)');grid on;title('Stern plane');
subplot(3,1,3);plot(t,rad2deg(log.ui(1,1:k)),'LineWidth',lw);
yline(20,'--r');yline(-20,'--r');ylabel('δ_r (°)');grid on;title('Rudder');
fprintf('viz_final: figures generated.\n');
