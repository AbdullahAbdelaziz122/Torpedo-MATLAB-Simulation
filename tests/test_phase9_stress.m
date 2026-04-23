%% test_phase9_stress.m  —  SMC vs PID in Severe Conditions
% SCENARIO: AUV commanded to track a straight line North (X increasing, Y=0)
% DISTURBANCE: 0.8 m/s current from the East, 1.5m waves.

fprintf('\n========================================\n');
fprintf('  Phase 9 STRESS TEST — Broadside Current & Waves\n');
fprintf('========================================\n\n');

if ~exist('auv','var'), error('Run ''buses; auv_params; auv_params_env_patch'' first.'); end

% Run simulations (60 seconds)
T_sim = 60;
fprintf('Running PID under stress...\n');
[t_pid, X_pid, ye_pid, ui_pid] = run_stress_sim(T_sim, 'pid', auv);

fprintf('Running SMC under stress...\n');
[t_smc, X_smc, ye_smc, ui_smc] = run_stress_sim(T_sim, 'smc', auv);

% Save results for plotting
save('phase9_stress_results.mat', 't_pid','X_pid','ye_pid','ui_pid', ...
                                  't_smc','X_smc','ye_smc','ui_smc');

fprintf('\nStress test complete. Run ''plot_phase9_stress'' to see results.\n');

% =========================================================================
% Runner Function
% =========================================================================
function [t_out, X_out, ye_out, ui_out] = run_stress_sim(T, ctrl_type, auv)
    dt = auv.sim.Ts;
    tvec = 0:dt:T; N = numel(tvec);
    
    X_out = zeros(N,12); ye_out = zeros(1,N); ui_out = zeros(3,N);
    x = zeros(12,1); x(1) = 1.5; x(9) = 5; % Start at 1.5 m/s, 5m depth
    X_out(1,:) = x';
    
    if strcmp(ctrl_type,'smc'), cs = control_smc_init_local(auv);
    else,                       cs = ctrl_pid_init_local(auv); end
    
    % Initialize Environment (with user's workspace overrides)
    es.Vc = auv.env.Vc_mean; es.betaVc = auv.env.betaVc_mean; es.w_c = 0;
    es.Vc_gm = es.Vc; es.beta_gm = es.betaVc; es.dt = dt;
    es.mu_Vc=0.01; es.mu_beta=0.005; es.sigma_Vc=0.02; es.sigma_beta=0.01;
    es.wave_on = auv.env.wave_on; es.wave_x = zeros(6,1);
    es.wf_Z = wf_i(auv.env.Hs, auv.env.Tp, auv.env.wave_scale_Z);
    es.wf_K = wf_i(auv.env.Hs*0.3, auv.env.Tp, auv.env.wave_scale_K);
    es.wf_M = wf_i(auv.env.Hs*0.5, auv.env.Tp, auv.env.wave_scale_M);
    
    los.ky = 1; los.kz = 1; los.delta_y = 10; los.delta_z = 10;
    los.U_nom = 1.5; los.U_max = 2.5; los.U_min = 0.5;

    for k = 2:N
        t = tvec(k);
        nu  = x(1:6); eta = x(7:12);
        eta(4:6) = arrayfun(@(a) atan2(sin(a),cos(a)), eta(4:6));
        
        % Environment step
        [Vc, bVc, wc, tau_env, es] = env_step_local(es, t);
        
        % Path: Straight North (X=1.5*t, Y=0, Z=5)
        pt = [1.5*t; 0; 5]; pv = [1.5; 0; 0]; ds = [1.5; 0; 0];
        
        % LOS Guidance
        [cd, ud_a, ud_s, cv, uv, errs] = los_step_local(nu, eta, pt, pv, ds, los);
        guid.chi_d=cd; guid.upsilon_d=ud_a; guid.ud=ud_s; guid.z_des=pt(3);
        
        % Control
        if strcmp(ctrl_type,'smc')
            [tc, nd, dbg, cs] = control_smc_step_local(guid, nu, eta, cs);
        else
            [tc, nd, dbg, cs] = ctrl_pid_step_local(guid, nu, eta, cs);
        end
        
        % Actuation & Physics
        ui = actuation_local(tc, nd, nu(1), auv);
        x  = rk4_local(x, ui, Vc, bVc, wc, tau_env, dt);
        
        X_out(k,:) = x'; ye_out(k) = errs.y_e; ui_out(:,k) = ui;
    end
    t_out = tvec;
end

% =========================================================================
% Local Functions (SMC with 2nd-order & dimensional fixes)
% =========================================================================
function cs=control_smc_init_local(auv)
    cs.dt=auv.sim.Ts;
    cs.u.lambda=0.20;cs.u.k=1.50;cs.u.phi=0.10;cs.u.integral=0;cs.u.e_prev=0;cs.u.sat=1000;
    cs.z.integral=0;cs.z.Kp=auv.ctrl.z.Kp;cs.z.Ki=auv.ctrl.z.Ki;cs.z.theta_max=auv.ctrl.z.theta_d_max;
    cs.theta.lambda=5.0;cs.theta.k=2.0;cs.theta.phi=0.05;cs.theta.integral=0;
    cs.theta.e_prev=0;cs.theta.sat=auv.ctrl.theta.sat;
    cs.psi.lambda=3.0;cs.psi.k=2.0;cs.psi.phi=0.10;cs.psi.integral=0;
    cs.psi.e_prev=0;cs.psi.sat=auv.ctrl.psi.sat;
    [~,~,M]=remus100();cs.m11=M(1,1);cs.m55=M(5,5);cs.m66=M(6,6);
    cs.m35=M(3,5);cs.m26=M(2,6);cs.W=auv.phys.W;cs.B=auv.phys.B;
    cs.zg=auv.phys.r_bG(3);cs.zb=auv.phys.r_bB(3);
end

function [tc,nd,dbg,cs]=control_smc_step_local(guid,nu,eta,cs)
    u=nu(1);v=nu(2);w=nu(3);q=nu(5);r=nu(6);theta=eta(5);
    chi_v=atan2(sin(eta(6)),cos(eta(6)));Ts=cs.dt;
    
    % Surge
    e_u=guid.ud-u; [s_u,cs.u]=smc_surface_local(e_u,cs.u,Ts);
    tau_X = cs.m11*(cs.u.lambda*e_u + cs.u.k*sat_func(s_u/cs.u.phi)) + cs.m11*(v*r-w*q);
    nd = max(0,min(1525,(1525/20)*tau_X));
    
    % Depth
    out_z=cs.z.Kp*(guid.z_des-eta(3))+cs.z.Ki*cs.z.integral;
    theta_d=max(-cs.z.theta_max,min(cs.z.theta_max,out_z));
    if abs(out_z)<=cs.z.theta_max, cs.z.integral=cs.z.integral+(guid.z_des-eta(3))*Ts; end
    
    % Pitch (2nd Order Surface)
    e_th=atan2(sin(theta_d-theta),cos(theta_d-theta)); e_th_dot = -q;
    s_th = e_th_dot + cs.theta.lambda*e_th;
    th_smc = max(-cs.theta.sat,min(cs.theta.sat, cs.theta.lambda*e_th_dot + cs.theta.k*sat_func(s_th/cs.theta.phi)));
    tau_M = cs.m55*th_smc + (cs.W*cs.zg-cs.B*cs.zb)*sin(theta) + 0.3*cs.m55*q - cs.m35*u*w;
    
    % Heading (2nd Order Surface)
    e_chi=atan2(sin(guid.chi_d-chi_v),cos(guid.chi_d-chi_v)); e_chi_dot = -r;
    s_psi = e_chi_dot + cs.psi.lambda*e_chi;
    psi_smc = max(-cs.psi.sat,min(cs.psi.sat, cs.psi.lambda*e_chi_dot + cs.psi.k*sat_func(s_psi/cs.psi.phi)));
    tau_N = cs.m66*psi_smc + 0.1*cs.m66*r + cs.m26*u*v;
    
    tc=zeros(6,1); tc(5)=tau_M; tc(6)=tau_N; dbg.dummy=0;
end

% Local PID Controller (Unchanged)
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

% Actuation with Lever Arms fixed
function ui=actuation_local(tau_ctrl,n_direct,U,auv)
    rho=auv.phys.rho; U_e=max(U,0.3); x_r=auv.act.x_r; x_s=auv.act.x_s;
    dr=tau_ctrl(6)/(-0.5*rho*U_e^2*auv.act.A_r*auv.act.CL_delta_r*x_r);
    ds=tau_ctrl(5)/(0.5*rho*U_e^2*auv.act.A_s*auv.act.CL_delta_s*x_s);
    dr=max(auv.act.delta_min,min(auv.act.delta_max,dr));
    ds=max(auv.act.delta_min,min(auv.act.delta_max,ds));
    n=max(auv.act.n_min,min(auv.act.n_max,n_direct));
    ui=[dr;ds;n];
end

% Physics & Math helpers
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

% Environment Helpers (Simplified from environment_lib.m)
function wf=wf_i(Hs,Tp,sc), if Hs<=0, wf.Kw=0; return; end; wf.omega_n=2*pi/Tp; wf.zeta=0.1; wf.Kw=sc*(Hs/4)*sqrt(2*wf.zeta*wf.omega_n); end
function [Vc,bVc,wc,tau,es]=env_step_local(es,t)
    dt=es.dt; es.Vc_gm=es.Vc_gm+dt*(-es.mu_Vc*es.Vc_gm+es.sigma_Vc*randn); Vc=max(0,es.Vc_gm);
    es.beta_gm=es.beta_gm+dt*(-es.mu_beta*es.beta_gm+es.sigma_beta*randn); bVc=es.beta_gm; wc=es.w_c;
    tau=zeros(6,1);
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
    xe=dx; ye=dy; ze=dz; % Simplified for perfectly North path
    pr=tanh(-los.ky*ye/los.delta_y); tr=tanh(los.kz*ze/los.delta_z);
    cd=pr; ud_a=tr; ud_s=1.5; errs.x_e=xe; errs.y_e=ye; errs.z_e=ze; cv=0; uv=0;
end