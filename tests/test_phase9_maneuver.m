%% test_phase9_maneuver.m  —  Torpedo Corkscrew Dive & Evasive Pull-out
% SCENARIO: 
%   0-10s:  Straight sprint at 2.0 m/s at 5m depth.
%   10-40s: High-speed spiral dive down to 20m depth (testing cross-coupling).
%   40-60s: Instant pull-out to straight-and-level flight at 20m depth.
% DISTURBANCE: 0.3 m/s current, 1.0m waves.

fprintf('\n======================================================\n');
fprintf('  Phase 9 STRESS TEST — Spiral Dive & Evasive Maneuver\n');
fprintf('======================================================\n\n');

if ~exist('auv','var'), error('Run ''buses; auv_params; auv_params_env_patch'' first.'); end

% Apply challenging environment
auv.env.Vc_mean = 0.3; auv.env.betaVc_mean = pi/4; 
auv.env.wave_on = true; auv.env.Hs = 1.0; auv.env.Tp = 6.0;

% Run simulations (60 seconds)
T_sim = 60;
fprintf('Running PID through maneuver...\n');
[t_pid, X_pid, ui_pid, pt_hist] = run_maneuver_sim(T_sim, 'pid', auv);

fprintf('Running SMC through maneuver...\n');
[t_smc, X_smc, ui_smc, ~] = run_maneuver_sim(T_sim, 'smc', auv);

% Save results for plotting
save('phase9_maneuver_results.mat', 't_pid','X_pid','ui_pid','pt_hist', ...
                                    't_smc','X_smc','ui_smc');

fprintf('\nManeuver test complete. Run ''plot_phase9_maneuver'' to see results.\n');

% =========================================================================
% Runner Function
% =========================================================================
function [t_out, X_out, ui_out, pt_out] = run_maneuver_sim(T, ctrl_type, auv)
    dt = auv.sim.Ts;
    tvec = 0:dt:T; N = numel(tvec);
    
    X_out = zeros(N,12); ui_out = zeros(3,N); pt_out = zeros(3,N);
    x = zeros(12,1); x(1) = 2.0; x(9) = 5; % Start at 2.0 m/s, 5m depth
    X_out(1,:) = x';
    
    if strcmp(ctrl_type,'smc'), cs = control_smc_init_local(auv);
    else,                       cs = ctrl_pid_init_local(auv); end
    
    es.Vc = auv.env.Vc_mean; es.betaVc = auv.env.betaVc_mean; es.w_c = 0;
    es.Vc_gm = es.Vc; es.beta_gm = es.betaVc; es.dt = dt;
    es.mu_Vc=0; es.mu_beta=0; es.sigma_Vc=0; es.sigma_beta=0;
    es.wave_on = auv.env.wave_on; es.wave_x = zeros(6,1);
    es.wf_Z = wf_i(auv.env.Hs, auv.env.Tp, auv.env.wave_scale_Z);
    es.wf_K = wf_i(auv.env.Hs*0.3, auv.env.Tp, auv.env.wave_scale_K);
    es.wf_M = wf_i(auv.env.Hs*0.5, auv.env.Tp, auv.env.wave_scale_M);
    
    los.ky = 1.5; los.kz = 1.5; los.delta_y = 10; los.delta_z = 10;
    los.U_nom = 2.0; los.U_max = 2.5; los.U_min = 0.5;

    for k = 2:N
        t = tvec(k);
        nu  = x(1:6); eta = x(7:12);
        eta(4:6) = arrayfun(@(a) atan2(sin(a),cos(a)), eta(4:6));
        
        [Vc, bVc, wc, tau_env, es] = env_step_local(es, t);
        
        % --- DYNAMIC PATH GENERATOR (The Maneuver) ---
        if t <= 10
            % Sprint straight
            pt = [2.0*t; 0; 5]; 
            pv = [2.0; 0; 0];
        elseif t <= 40
            % Corkscrew Dive
            tau = t - 10; R = 20; w = 0.1;
            pt = [20 + R*sin(w*tau); R*(1 - cos(w*tau)); 5 + 0.5*tau];
            pv = [R*w*cos(w*tau); R*w*sin(w*tau); 0.5];
        else
            % Evasive pull-out and level off
            tau_exit = 30; R = 20; w = 0.1;
            x_exit = 20 + R*sin(w*tau_exit); y_exit = R*(1 - cos(w*tau_exit));
            vx_exit = R*w*cos(w*tau_exit);   vy_exit = R*w*sin(w*tau_exit);
            dt_exit = t - 40;
            pt = [x_exit + vx_exit*dt_exit; y_exit + vy_exit*dt_exit; 20];
            pv = [vx_exit; vy_exit; 0];
        end
        ds = pv; % Desired speed vector matches path velocity
        pt_out(:,k) = pt;

        [cd, ud_a, ud_s, ~, ~, ~] = los_step_local(nu, eta, pt, pv, ds, los);
        guid.chi_d=cd; guid.upsilon_d=ud_a; guid.ud=ud_s; guid.z_des=pt(3);
        
        if strcmp(ctrl_type,'smc')
            [tc, nd, ~, cs] = control_smc_step_local(guid, nu, eta, cs);
        else
            [tc, nd, ~, cs] = ctrl_pid_step_local(guid, nu, eta, cs);
        end
        
        ui = actuation_local(tc, nd, nu(1), auv);
        x  = rk4_local(x, ui, Vc, bVc, wc, tau_env, dt);
        
        X_out(k,:) = x'; ui_out(:,k) = ui;
    end
    t_out = tvec;
end

% =========================================================================
% Local Functions (SMC with exact 2nd-order & dimension fixes)
% =========================================================================
function cs=control_smc_init_local(auv)
    cs.dt=auv.sim.Ts;
    cs.u.lambda=0.20;cs.u.k=1.50;cs.u.phi=0.10;cs.u.integral=0;cs.u.e_prev=0;cs.u.sat=1000;
    cs.z.integral=0;cs.z.Kp=auv.ctrl.z.Kp;cs.z.Ki=auv.ctrl.z.Ki;cs.z.theta_max=auv.ctrl.z.theta_d_max;
    cs.theta.lambda=5.0;cs.theta.k=2.0;cs.theta.phi=0.05;cs.theta.integral=0;cs.theta.sat=auv.ctrl.theta.sat;
    cs.psi.lambda=3.0;cs.psi.k=2.0;cs.psi.phi=0.10;cs.psi.integral=0;cs.psi.sat=auv.ctrl.psi.sat;
    [~,~,M]=remus100();cs.m11=M(1,1);cs.m55=M(5,5);cs.m66=M(6,6);
    cs.m35=M(3,5);cs.m26=M(2,6);cs.W=auv.phys.W;cs.B=auv.phys.B;
    cs.zg=auv.phys.r_bG(3);cs.zb=auv.phys.r_bB(3);
end

function [tc,nd,dbg,cs]=control_smc_step_local(guid,nu,eta,cs)
    u=nu(1);v=nu(2);w=nu(3);q=nu(5);r=nu(6);theta=eta(5); chi_v=atan2(sin(eta(6)),cos(eta(6)));Ts=cs.dt;
    
    % Surge (Dimensional Fix)
    e_u=guid.ud-u; [s_u,cs.u]=smc_surface_local(e_u,cs.u,Ts);
    tau_X = cs.m11*(cs.u.lambda*e_u + cs.u.k*sat_func(s_u/cs.u.phi)) + cs.m11*(v*r-w*q);
    nd = max(0,min(1525,(1525/20)*tau_X));
    
    % Depth outer PI
    out_z=cs.z.Kp*(guid.z_des-eta(3))+cs.z.Ki*cs.z.integral;
    theta_d=max(-cs.z.theta_max,min(cs.z.theta_max,out_z));
    if abs(out_z)<=cs.z.theta_max, cs.z.integral=cs.z.integral+(guid.z_des-eta(3))*Ts; end
    
    % Pitch (2nd Order Fix)
    e_th=atan2(sin(theta_d-theta),cos(theta_d-theta)); e_th_dot = -q;
    s_th = e_th_dot + cs.theta.lambda*e_th;
    th_smc = max(-cs.theta.sat,min(cs.theta.sat, cs.theta.lambda*e_th_dot + cs.theta.k*sat_func(s_th/cs.theta.phi)));
    tau_M = cs.m55*th_smc + (cs.W*cs.zg-cs.B*cs.zb)*sin(theta) + 0.3*cs.m55*q - cs.m35*u*w;
    
    % Heading (2nd Order Fix)
    e_chi=atan2(sin(guid.chi_d-chi_v),cos(guid.chi_d-chi_v)); e_chi_dot = -r;
    s_psi = e_chi_dot + cs.psi.lambda*e_chi;
    psi_smc = max(-cs.psi.sat,min(cs.psi.sat, cs.psi.lambda*e_chi_dot + cs.psi.k*sat_func(s_psi/cs.psi.phi)));
    tau_N = cs.m66*psi_smc + 0.1*cs.m66*r + cs.m26*u*v;
    
    tc=zeros(6,1); tc(5)=tau_M; tc(6)=tau_N; dbg.dummy=0;
end

function cs=ctrl_pid_init_local(auv)
    cs.dt=auv.sim.Ts;
    cs.u.integral=0;cs.u.e_prev=0;cs.u.Kp=auv.ctrl.u.Kp; cs.u.Ki=auv.ctrl.u.Ki;cs.u.Kd=auv.ctrl.u.Kd;cs.u.sat=auv.ctrl.u.u_max;
    cs.z.integral=0;cs.z.Kp=auv.ctrl.z.Kp;cs.z.Ki=auv.ctrl.z.Ki;cs.z.theta_max=auv.ctrl.z.theta_d_max;
    cs.theta.integral=0;cs.theta.e_prev=0;cs.theta.Kp=auv.ctrl.theta.Kp; cs.theta.Ki=auv.ctrl.theta.Ki;cs.theta.Kd=auv.ctrl.theta.Kd;cs.theta.sat=auv.ctrl.theta.sat;
    cs.psi.integral=0;cs.psi.e_prev=0;cs.psi.Kp=auv.ctrl.psi.Kp; cs.psi.Ki=auv.ctrl.psi.Ki;cs.psi.Kd=auv.ctrl.psi.Kd;cs.psi.sat=auv.ctrl.psi.sat;
    [~,~,M]=remus100();cs.m11=M(1,1);cs.m55=M(5,5);cs.m66=M(6,6);cs.m35=M(3,5);cs.m26=M(2,6);cs.W=auv.phys.W;cs.B=auv.phys.B;cs.zg=auv.phys.r_bG(3);cs.zb=auv.phys.r_bB(3);
end

function [tc,nd,dbg,cs]=ctrl_pid_step_local(guid,nu,eta,cs)
    u=nu(1);v=nu(2);w=nu(3);q=nu(5);r=nu(6);theta=eta(5); chi_v=atan2(sin(eta(6)),cos(eta(6)));Ts=cs.dt;
    e_u=guid.ud-u;du=(e_u-cs.u.e_prev)/Ts;
    out_u=cs.u.Kp*e_u+cs.u.Ki*cs.u.integral+cs.u.Kd*du; nd=max(0,min(1525,(1525/20)*out_u));
    if abs(out_u)<=cs.u.sat,cs.u.integral=cs.u.integral+e_u*Ts;end;cs.u.e_prev=e_u;
    out_z=cs.z.Kp*(guid.z_des-eta(3))+cs.z.Ki*cs.z.integral; theta_d=max(-cs.z.theta_max,min(cs.z.theta_max,out_z));
    if abs(out_z)<=cs.z.theta_max,cs.z.integral=cs.z.integral+(guid.z_des-eta(3))*Ts;end
    e_th=atan2(sin(theta_d-theta),cos(theta_d-theta)); dth=(e_th-cs.theta.e_prev)/Ts;
    out_th_r=cs.theta.Kp*e_th+cs.theta.Ki*cs.theta.integral+cs.theta.Kd*dth; out_th=max(-cs.theta.sat,min(cs.theta.sat,out_th_r));
    tau_M=cs.m55*out_th+(cs.W*cs.zg-cs.B*cs.zb)*sin(theta)+0.3*cs.m55*q-cs.m35*u*w;
    if abs(out_th_r)<=cs.theta.sat,cs.theta.integral=cs.theta.integral+e_th*Ts;end;cs.theta.e_prev=e_th;
    e_chi=atan2(sin(guid.chi_d-chi_v),cos(guid.chi_d-chi_v)); dpsi=(e_chi-cs.psi.e_prev)/Ts;
    out_p_r=cs.psi.Kp*e_chi+cs.psi.Ki*cs.psi.integral+cs.psi.Kd*dpsi; out_p=max(-cs.psi.sat,min(cs.psi.sat,out_p_r));
    tau_N=cs.m66*out_p+0.1*cs.m66*r+cs.m26*u*v;
    if abs(out_p_r)<=cs.psi.sat,cs.psi.integral=cs.psi.integral+e_chi*Ts;end;cs.psi.e_prev=e_chi;
    tc=zeros(6,1);tc(5)=tau_M;tc(6)=tau_N;dbg.dummy=0;
end

% Actuation (Lever Arm Fixes)
function ui=actuation_local(tau_ctrl,n_direct,U,auv)
    rho=auv.phys.rho; U_e=max(U,0.3); x_r=auv.act.x_r; x_s=auv.act.x_s;
    dr=tau_ctrl(6)/(-0.5*rho*U_e^2*auv.act.A_r*auv.act.CL_delta_r*x_r);
    ds=tau_ctrl(5)/(0.5*rho*U_e^2*auv.act.A_s*auv.act.CL_delta_s*x_s);
    dr=max(auv.act.delta_min,min(auv.act.delta_max,dr));
    ds=max(auv.act.delta_min,min(auv.act.delta_max,ds));
    n=max(auv.act.n_min,min(auv.act.n_max,n_direct));
    ui=[dr;ds;n];
end

function x_n=rk4_local(x,ui,Vc,bVc,wc,tau_env,dt)
    [~,~,M]=remus100();a_env=[M\tau_env(1:6);zeros(6,1)];
    f=@(xx) remus100(xx,ui,Vc,bVc,wc)+a_env;
    k1=f(x);k2=f(x+(dt/2)*k1);k3=f(x+(dt/2)*k2);k4=f(x+dt*k3);
    x_n=x+(dt/6)*(k1+2*k2+2*k3+k4);
end
function [s,ch]=smc_surface_local(error,ch,dt)
    ch.integral=ch.integral+error*dt; s=error+ch.lambda*ch.integral;
    if abs(s)>3*ch.phi,ch.integral=ch.integral-error*dt;end; ch.e_prev=error;
end
function y=sat_func(x), if abs(x)<=1,y=x;else,y=sign(x);end, end

function wf=wf_i(Hs,Tp,sc), if Hs<=0, wf.Kw=0; return; end; wf.omega_n=2*pi/Tp; wf.zeta=0.1; wf.Kw=sc*(Hs/4)*sqrt(2*wf.zeta*wf.omega_n); end
function [Vc,bVc,wc,tau,es]=env_step_local(es,t)
    Vc=es.Vc; bVc=es.betaVc; wc=es.w_c; dt=es.dt; tau=zeros(6,1);
    if es.wave_on
        [tau(3),es.wave_x(1:2)]=wf_step(es.wave_x(1:2),es.wf_Z,dt);
        [tau(4),es.wave_x(3:4)]=wf_step(es.wave_x(3:4),es.wf_K,dt);
        [tau(5),es.wave_x(5:6)]=wf_step(es.wave_x(5:6),es.wf_M,dt);
    end
end
function [o,xn]=wf_step(x,wf,dt)
    dx1=x(2); dx2=-wf.omega_n^2*x(1)-2*wf.zeta*wf.omega_n*x(2)+wf.Kw*randn;
    xn=[x(1)+dt*dx1; x(2)+dt*dx2]; o=xn(1);
end
function [cd,ud_a,ud_s,cv,uv,errs]=los_step_local(nu,eta,pt,pv,ds,los)
    dx=eta(1)-pt(1);dy=eta(2)-pt(2);dz=eta(3)-pt(3);
    pr=tanh(-los.ky*dy/los.delta_y); tr=tanh(los.kz*dz/los.delta_z);
    
    % Full 3D LOS
    if norm(pv(1:2))<1e-6,pp=0;else,pp=atan2(pv(2),pv(1));end
    h2=sqrt(pv(1)^2+pv(2)^2);if h2<1e-6,tp=0;else,tp=atan(-pv(3)/h2);end
    cp=cos(pp);sp=sin(pp);ct=cos(tp);st=sin(tp);
    xe=cp*ct*dx+sp*ct*dy-st*dz;ye=-sp*dx+cp*dy;ze=cp*st*dx+sp*st*dy+ct*dz;
    
    arg=max(-1,min(1,sin(tp)*cos(tr)*cos(pr)+cos(tp)*sin(tr))); ud_a=asin(arg);
    cy=cos(pp)*sin(pr)*cos(tr)-sin(tp)*sin(tr)*sin(pp)+sin(pp)*cos(pr)*cos(tp)*cos(tr);
    cx=-sin(pp)*sin(pr)*cos(tr)-sin(tp)*sin(tr)*cos(pp)+cos(pp)*cos(pr)*cos(tp)*cos(tr);
    cd=atan2(cy,cx); ud_s=los.U_nom; errs.x_e=xe; errs.y_e=ye; errs.z_e=ze; cv=0; uv=0;
end