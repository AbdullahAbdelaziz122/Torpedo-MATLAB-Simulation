%% test_phase10_spiral_maneuver.m  —  SMC vs PID on Spiral + Torpedo Maneuver
% SCENARIO:
%   Phase A (0 - T_switch): expanding descending spiral search pattern
%   Phase B (T_switch - T_sim): aggressive weaving terminal run
% PURPOSE:
%   Compare PID and SMC under a more realistic torpedo-like operation where
%   heading, cross-track, depth, and actuator effort are all excited.
%
% USAGE:
%   Run:  buses; auv_params; auv_params_env_patch
%   Then: test_phase10_spiral_maneuver
%   Then: plot_phase10_spiral_maneuver

fprintf('\n=======================================================\n');
fprintf('  Phase 10 TEST — Spiral Search + Torpedo Maneuver\n');
fprintf('=======================================================\n\n');

if ~exist('auv','var')
    error('Run ''buses; auv_params; auv_params_env_patch'' first.');
end

% Simulation horizon
T_sim    = 90;
T_switch = 45;

fprintf('Running PID on spiral + maneuver scenario...\n');
[t_pid, X_pid, err_pid, ui_pid, ref_pid, phase_pid] = run_torpedo_sim(T_sim, T_switch, 'pid', auv);

fprintf('Running SMC on spiral + maneuver scenario...\n');
[t_smc, X_smc, err_smc, ui_smc, ref_smc, phase_smc] = run_torpedo_sim(T_sim, T_switch, 'smc', auv);

% Save for plotting
save('phase10_spiral_maneuver_results.mat', ...
     'T_sim','T_switch', ...
     't_pid','X_pid','err_pid','ui_pid','ref_pid','phase_pid', ...
     't_smc','X_smc','err_smc','ui_smc','ref_smc','phase_smc');

fprintf('\nPhase 10 test complete. Run ''plot_phase10_spiral_maneuver'' to see results.\n');

% =========================================================================
% Main runner
% =========================================================================
function [t_out, X_out, err_out, ui_out, ref_out, phase_out] = run_torpedo_sim(T, T_switch, ctrl_type, auv)
    dt = auv.sim.Ts;
    tvec = 0:dt:T;
    N    = numel(tvec);

    X_out     = zeros(N,12);
    err_out   = zeros(N,3);   % [x_e, y_e, z_e]
    ui_out    = zeros(3,N);   % [rudder; stern plane; prop]
    ref_out   = zeros(N,5);   % [x_ref, y_ref, z_ref, chi_ref, u_ref]
    phase_out = zeros(N,1);

    % Initial state: modest forward speed, level start, shallow depth
    x = zeros(12,1);
    x(1) = 1.5;   % surge speed
    x(9) = 5.0;   % depth [m]
    X_out(1,:) = x.';

    [pt0, pv0, ds0, phase0] = torpedo_reference_local(0, T_switch);
    chi_ref0     = atan2(ds0(2), ds0(1));
    ref_out(1,:) = [pt0(:).', chi_ref0, norm(pv0)];
    phase_out(1) = phase0;

    if strcmpi(ctrl_type,'smc')
        cs = control_smc_init_local(auv);
    else
        cs = ctrl_pid_init_local(auv);
    end

    % Scenario environment: broadside current + waves + stochastic variation
    es = scenario_env_init_local(auv, dt);

    los.ky      = 1.2;
    los.kz      = 1.0;
    los.delta_y = 8.0;
    los.delta_z = 8.0;
    los.U_nom   = 1.8;
    los.U_max   = 2.4;
    los.U_min   = 0.6;

    for k = 2:N
        t   = tvec(k);
        nu  = x(1:6);
        eta = x(7:12);
        eta(4:6) = arrayfun(@(a) atan2(sin(a),cos(a)), eta(4:6));

        % Environment step
        [Vc, bVc, wc, tau_env, es] = env_step_local(es, t, T_switch);

        % Reference path generator (spiral -> terminal torpedo maneuver)
        [pt, pv, ds, phase_id] = torpedo_reference_local(t, T_switch);

        % Guidance
        [cd, ud_a, ud_s, cv, uv, errs] = los_step_local(nu, eta, pt, pv, ds, los);
        guid.chi_d     = cd;
        guid.upsilon_d = ud_a;
        guid.ud        = ud_s;
        guid.z_des     = pt(3);

        % Controller
        if strcmpi(ctrl_type,'smc')
            [tc, nd, dbg, cs] = control_smc_step_local(guid, nu, eta, cs); %#ok<ASGLU>
        else
            [tc, nd, dbg, cs] = ctrl_pid_step_local(guid, nu, eta, cs); %#ok<ASGLU>
        end

        % Actuation and vehicle propagation
        ui = actuation_local(tc, nd, nu(1), auv);
        x  = rk4_local(x, ui, Vc, bVc, wc, tau_env, dt);

        chi_ref = atan2(ds(2), ds(1));
        X_out(k,:)   = x.';
        err_out(k,:) = [errs.x_e, errs.y_e, errs.z_e];
        ui_out(:,k)  = ui;
        ref_out(k,:) = [pt(:).', chi_ref, ud_s];
        phase_out(k) = phase_id;
    end

    err_out(1,:) = initial_error_local(X_out(1,7:9).', ref_out(1,1:3).', ref_out(1,4));
    t_out = tvec;
end

% =========================================================================
% Reference trajectory
% =========================================================================
function [pt, pv, ds, phase_id] = torpedo_reference_local(t, T_switch)
    if t <= T_switch
        % Phase A: expanding descending spiral search
        phase_id = 1;

        R0      = 2.0;
        Rdot    = 0.10;
        omega   = 0.22;
        z0      = 5.0;
        zdot    = 0.06;
        theta   = omega * t;
        R       = R0 + Rdot * t;

        pt = [R*cos(theta);
              R*sin(theta);
              z0 + zdot*t];

        pv = [Rdot*cos(theta) - R*omega*sin(theta);
              Rdot*sin(theta) + R*omega*cos(theta);
              zdot];

        ds = pv;
    else
        % Phase B: terminal torpedo-like weaving maneuver
        phase_id = 2;
        [pt_sw, pv_sw] = spiral_endpoint_local(T_switch);

        tau  = t - T_switch;
        U    = 2.2;    % aggressive forward run
        A_y  = 8.0;    % weave amplitude [m]
        w_y  = 0.42;   % weave rate [rad/s]
        A_z  = 1.2;    % depth maneuver amplitude [m]
        w_z  = 0.18;

        pt = [pt_sw(1) + U*tau;
              pt_sw(2) + A_y*sin(w_y*tau);
              pt_sw(3) + A_z*sin(w_z*tau)];

        pv = [U;
              A_y*w_y*cos(w_y*tau);
              A_z*w_z*cos(w_z*tau)];

        ds = pv;
    end
end

function e0 = initial_error_local(p, pref, chi_ref)
    dxy = p(1:2) - pref(1:2);
    Rz  = [ cos(chi_ref) sin(chi_ref);
           -sin(chi_ref) cos(chi_ref)];
    epf = Rz * dxy;
    e0  = [epf(1), epf(2), p(3)-pref(3)];
end

function [pt, pv] = spiral_endpoint_local(t)
    R0    = 2.0;
    Rdot  = 0.10;
    omega = 0.22;
    z0    = 5.0;
    zdot  = 0.06;
    theta = omega * t;
    R     = R0 + Rdot * t;

    pt = [R*cos(theta);
          R*sin(theta);
          z0 + zdot*t];

    pv = [Rdot*cos(theta) - R*omega*sin(theta);
          Rdot*sin(theta) + R*omega*cos(theta);
          zdot];
end

% =========================================================================
% Scenario environment
% =========================================================================
function es = scenario_env_init_local(auv, dt)
    es.dt      = dt;
    es.Vc      = max(0.7, auv.env.Vc_mean); % stronger than nominal mission
    es.betaVc  = pi/2;                      % broadside current from East
    es.w_c     = 0;

    es.Vc_gm   = es.Vc;
    es.beta_gm = es.betaVc;
    es.mu_Vc   = 0.020;
    es.mu_beta = 0.010;
    es.sigma_Vc   = 0.030;
    es.sigma_beta = 0.015;

    es.wave_on = auv.env.wave_on;
    es.wave_x  = zeros(6,1);

    % Slightly more energetic than the phase 9 test
    Hs_eff = max(auv.env.Hs, 1.2);
    Tp_eff = max(auv.env.Tp, 5.5);
    es.wf_Z = wf_i(Hs_eff,        Tp_eff, auv.env.wave_scale_Z);
    es.wf_K = wf_i(Hs_eff * 0.35, Tp_eff, auv.env.wave_scale_K);
    es.wf_M = wf_i(Hs_eff * 0.55, Tp_eff, auv.env.wave_scale_M);
end

% =========================================================================
% Controllers (same structure as phase 9)
% =========================================================================
function cs = control_smc_init_local(auv)
    cs.dt = auv.sim.Ts;
    cs.u.lambda=0.20; cs.u.k=1.50; cs.u.phi=0.10; cs.u.integral=0; cs.u.e_prev=0; cs.u.sat=1000;
    cs.z.integral=0; cs.z.Kp=auv.ctrl.z.Kp; cs.z.Ki=auv.ctrl.z.Ki; cs.z.theta_max=auv.ctrl.z.theta_d_max;
    cs.theta.lambda=5.0; cs.theta.k=2.0; cs.theta.phi=0.05; cs.theta.integral=0;
    cs.theta.e_prev=0; cs.theta.sat=auv.ctrl.theta.sat;
    cs.psi.lambda=3.0; cs.psi.k=2.2; cs.psi.phi=0.10; cs.psi.integral=0;
    cs.psi.e_prev=0; cs.psi.sat=auv.ctrl.psi.sat;
    [~,~,M]=remus100();
    cs.m11=M(1,1); cs.m55=M(5,5); cs.m66=M(6,6);
    cs.m35=M(3,5); cs.m26=M(2,6);
    cs.W=auv.phys.W; cs.B=auv.phys.B; cs.zg=auv.phys.r_bG(3); cs.zb=auv.phys.r_bB(3);
end

function [tc,nd,dbg,cs] = control_smc_step_local(guid,nu,eta,cs)
    u=nu(1); v=nu(2); w=nu(3); q=nu(5); r=nu(6); theta=eta(5);
    chi_v = atan2(sin(eta(6)),cos(eta(6)));
    Ts = cs.dt;

    % Surge
    e_u = guid.ud - u;
    [s_u, cs.u] = smc_surface_local(e_u, cs.u, Ts);
    tau_X = cs.m11*(cs.u.lambda*e_u + cs.u.k*sat_func(s_u/cs.u.phi)) + cs.m11*(v*r - w*q);
    nd = max(0, min(1525, (1525/20)*tau_X));

    % Depth -> pitch reference
    out_z   = cs.z.Kp*(guid.z_des - eta(3)) + cs.z.Ki*cs.z.integral;
    theta_d = max(-cs.z.theta_max, min(cs.z.theta_max, out_z));
    if abs(out_z) <= cs.z.theta_max
        cs.z.integral = cs.z.integral + (guid.z_des - eta(3))*Ts;
    end

    % Pitch SMC
    e_th    = atan2(sin(theta_d-theta), cos(theta_d-theta));
    e_thdot = -q;
    s_th    = e_thdot + cs.theta.lambda*e_th;
    th_smc  = max(-cs.theta.sat, min(cs.theta.sat, cs.theta.lambda*e_thdot + cs.theta.k*sat_func(s_th/cs.theta.phi)));
    tau_M   = cs.m55*th_smc + (cs.W*cs.zg - cs.B*cs.zb)*sin(theta) + 0.3*cs.m55*q - cs.m35*u*w;

    % Heading SMC
    e_chi   = atan2(sin(guid.chi_d-chi_v), cos(guid.chi_d-chi_v));
    e_chdot = -r;
    s_psi   = e_chdot + cs.psi.lambda*e_chi;
    psi_smc = max(-cs.psi.sat, min(cs.psi.sat, cs.psi.lambda*e_chdot + cs.psi.k*sat_func(s_psi/cs.psi.phi)));
    tau_N   = cs.m66*psi_smc + 0.1*cs.m66*r + cs.m26*u*v;

    tc = zeros(6,1);
    tc(5) = tau_M;
    tc(6) = tau_N;
    dbg.dummy = 0;
end

function cs = ctrl_pid_init_local(auv)
    cs.dt = auv.sim.Ts;
    cs.u.integral=0; cs.u.e_prev=0; cs.u.Kp=auv.ctrl.u.Kp; cs.u.Ki=auv.ctrl.u.Ki; cs.u.Kd=auv.ctrl.u.Kd; cs.u.sat=auv.ctrl.u.u_max;
    cs.z.integral=0; cs.z.Kp=auv.ctrl.z.Kp; cs.z.Ki=auv.ctrl.z.Ki; cs.z.theta_max=auv.ctrl.z.theta_d_max;
    cs.theta.integral=0; cs.theta.e_prev=0; cs.theta.Kp=auv.ctrl.theta.Kp; cs.theta.Ki=auv.ctrl.theta.Ki; cs.theta.Kd=auv.ctrl.theta.Kd; cs.theta.sat=auv.ctrl.theta.sat;
    cs.psi.integral=0; cs.psi.e_prev=0; cs.psi.Kp=auv.ctrl.psi.Kp; cs.psi.Ki=auv.ctrl.psi.Ki; cs.psi.Kd=auv.ctrl.psi.Kd; cs.psi.sat=auv.ctrl.psi.sat;
    [~,~,M]=remus100();
    cs.m11=M(1,1); cs.m55=M(5,5); cs.m66=M(6,6); cs.m35=M(3,5); cs.m26=M(2,6);
    cs.W=auv.phys.W; cs.B=auv.phys.B; cs.zg=auv.phys.r_bG(3); cs.zb=auv.phys.r_bB(3);
end

function [tc,nd,dbg,cs] = ctrl_pid_step_local(guid,nu,eta,cs)
    u=nu(1); v=nu(2); w=nu(3); q=nu(5); r=nu(6); theta=eta(5);
    chi_v = atan2(sin(eta(6)),cos(eta(6)));
    Ts = cs.dt;

    % Surge PID
    e_u = guid.ud - u;
    du  = (e_u - cs.u.e_prev)/Ts;
    out_u = cs.u.Kp*e_u + cs.u.Ki*cs.u.integral + cs.u.Kd*du;
    nd = max(0, min(1525, (1525/20)*out_u));
    if abs(out_u) <= cs.u.sat
        cs.u.integral = cs.u.integral + e_u*Ts;
    end
    cs.u.e_prev = e_u;

    % Depth loop
    out_z   = cs.z.Kp*(guid.z_des - eta(3)) + cs.z.Ki*cs.z.integral;
    theta_d = max(-cs.z.theta_max, min(cs.z.theta_max, out_z));
    if abs(out_z) <= cs.z.theta_max
        cs.z.integral = cs.z.integral + (guid.z_des - eta(3))*Ts;
    end

    % Pitch PID
    e_th  = atan2(sin(theta_d-theta), cos(theta_d-theta));
    dth   = (e_th - cs.theta.e_prev)/Ts;
    out_th_r = cs.theta.Kp*e_th + cs.theta.Ki*cs.theta.integral + cs.theta.Kd*dth;
    out_th   = max(-cs.theta.sat, min(cs.theta.sat, out_th_r));
    tau_M    = cs.m55*out_th + (cs.W*cs.zg - cs.B*cs.zb)*sin(theta) + 0.3*cs.m55*q - cs.m35*u*w;
    if abs(out_th_r) <= cs.theta.sat
        cs.theta.integral = cs.theta.integral + e_th*Ts;
    end
    cs.theta.e_prev = e_th;

    % Heading PID
    e_chi = atan2(sin(guid.chi_d-chi_v), cos(guid.chi_d-chi_v));
    dpsi  = (e_chi - cs.psi.e_prev)/Ts;
    out_p_r = cs.psi.Kp*e_chi + cs.psi.Ki*cs.psi.integral + cs.psi.Kd*dpsi;
    out_p   = max(-cs.psi.sat, min(cs.psi.sat, out_p_r));
    tau_N   = cs.m66*out_p + 0.1*cs.m66*r + cs.m26*u*v;
    if abs(out_p_r) <= cs.psi.sat
        cs.psi.integral = cs.psi.integral + e_chi*Ts;
    end
    cs.psi.e_prev = e_chi;

    tc = zeros(6,1);
    tc(5) = tau_M;
    tc(6) = tau_N;
    dbg.dummy = 0;
end

% =========================================================================
% Actuation, physics, guidance, helpers
% =========================================================================
function ui = actuation_local(tau_ctrl,n_direct,U,auv)
    rho = auv.phys.rho;
    U_e = max(U,0.3);
    x_r = auv.act.x_r;
    x_s = auv.act.x_s;

    dr = tau_ctrl(6)/(-0.5*rho*U_e^2*auv.act.A_r*auv.act.CL_delta_r*x_r);
    ds = tau_ctrl(5)/( 0.5*rho*U_e^2*auv.act.A_s*auv.act.CL_delta_s*x_s);

    dr = max(auv.act.delta_min, min(auv.act.delta_max, dr));
    ds = max(auv.act.delta_min, min(auv.act.delta_max, ds));
    n  = max(auv.act.n_min,    min(auv.act.n_max,    n_direct));

    ui = [dr; ds; n];
end

function x_n = rk4_local(x,ui,Vc,bVc,wc,tau_env,dt)
    [~,~,M] = remus100();
    a_env = [M\tau_env(1:6); zeros(6,1)];
    f  = @(xx) remus100(xx,ui,Vc,bVc,wc) + a_env;
    k1 = f(x);
    k2 = f(x + (dt/2)*k1);
    k3 = f(x + (dt/2)*k2);
    k4 = f(x + dt*k3);
    x_n = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end

function [cd,ud_a,ud_s,cv,uv,errs] = los_step_local(~,eta,pt,pv,ds,los)
    p   = eta(1:3);
    dxy = p(1:2) - pt(1:2);

    chi_p = atan2(ds(2), ds(1));
    Rz    = [ cos(chi_p) sin(chi_p);
             -sin(chi_p) cos(chi_p)];
    e_pf  = Rz * dxy;

    x_e = e_pf(1);
    y_e = e_pf(2);
    z_e = p(3) - pt(3);

    chi_corr = atan(-los.ky * y_e / max(los.delta_y,1e-6));
    cd = atan2(sin(chi_p + chi_corr), cos(chi_p + chi_corr));

    ud_a = atan(-los.kz * z_e / max(los.delta_z,1e-6));
    ud_s = min(los.U_max, max(los.U_min, norm(pv)));

    errs.x_e = x_e;
    errs.y_e = y_e;
    errs.z_e = z_e;
    cv = 0;
    uv = 0;
end

function [s,ch] = smc_surface_local(error,ch,dt)
    ch.integral = ch.integral + error*dt;
    s = error + ch.lambda*ch.integral;
    if abs(s) > 3*ch.phi
        ch.integral = ch.integral - error*dt;
    end
    ch.e_prev = error;
end

function y = sat_func(x)
    if abs(x) <= 1
        y = x;
    else
        y = sign(x);
    end
end

function wf = wf_i(Hs,Tp,sc)
    if Hs <= 0
        wf.Kw = 0;
        wf.omega_n = 0;
        wf.zeta = 0.1;
        return;
    end
    wf.omega_n = 2*pi/Tp;
    wf.zeta    = 0.1;
    wf.Kw      = sc*(Hs/4)*sqrt(2*wf.zeta*wf.omega_n);
end

function [Vc,bVc,wc,tau,es] = env_step_local(es,t,T_switch)
    dt = es.dt;

    % Slow stochastic current variation
    es.Vc_gm = es.Vc_gm + dt*(-es.mu_Vc*es.Vc_gm + es.sigma_Vc*randn);
    es.beta_gm = es.beta_gm + dt*(-es.mu_beta*(es.beta_gm-es.betaVc) + es.sigma_beta*randn);

    % Add a transient current increase during the terminal phase
    Vc_burst = 0;
    if t > T_switch
        tau_m = t - T_switch;
        Vc_burst = 0.18*(1 - exp(-tau_m/5));
    end

    Vc  = max(0, es.Vc_gm + Vc_burst);
    bVc = es.beta_gm;
    wc  = es.w_c;
    tau = zeros(6,1);

    if es.wave_on
        [tau(3), es.wave_x(1:2)] = wf_step(es.wave_x(1:2), es.wf_Z, dt);
        [tau(4), es.wave_x(3:4)] = wf_step(es.wave_x(3:4), es.wf_K, dt);
        [tau(5), es.wave_x(5:6)] = wf_step(es.wave_x(5:6), es.wf_M, dt);
    end
end

function [o,xn] = wf_step(x,wf,dt)
    dx1 = x(2);
    dx2 = -wf.omega_n^2*x(1) - 2*wf.zeta*wf.omega_n*x(2) + wf.Kw*randn;
    xn  = [x(1)+dt*dx1; x(2)+dt*dx2];
    o   = xn(1);
end
