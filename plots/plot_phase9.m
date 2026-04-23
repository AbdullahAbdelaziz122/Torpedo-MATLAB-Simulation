%% plot_phase9.m  —  Phase 9: PID vs SMC Comparison Plots
%
% Generates side-by-side comparison of PID and SMC on identical scenario.
% Also shows the sliding surface convergence — the SMC-specific diagnostic.
%
% HOW TO RUN:
%   >> plot_phase9

if ~exist('phase9_results.mat','file')
    error('Run test_phase9_smc first.');
end
load('phase9_results.mat');

lw  = 1.5;  fs = 11;
c_pid = [0.13 0.47 0.71];   % blue for PID
c_smc = [0.84 0.15 0.16];   % red for SMC
c_ref = [0.5  0.5  0.5];
close all;

% =========================================================================
% Figure 1 — Heading step: PID vs SMC
% =========================================================================
figure(30);
set(gcf,'Name','Phase 9 — PID vs SMC heading','NumberTitle','off','Position',[50 450 900 380]);

subplot(1,3,1)
plot(t_pid, rad2deg(X_pid(:,12)),'Color',c_pid,'LineWidth',lw,'DisplayName','PID'); hold on;
plot(t_smc, rad2deg(X_smc(:,12)),'Color',c_smc,'LineWidth',lw,'DisplayName','SMC');
yline(45,'--','Color',c_ref,'LineWidth',1,'Label','45° target');
yline(42,':','Color',c_ref,'LineWidth',0.8); yline(48,':','Color',c_ref,'LineWidth',0.8);
xlabel('t (s)','FontSize',fs); ylabel('ψ (deg)','FontSize',fs);
title('Heading response','FontSize',fs); legend('Location','southeast','FontSize',9); grid on;

subplot(1,3,2)
plot(t_pid, rad2deg(e_chi_pid),'Color',c_pid,'LineWidth',lw,'DisplayName','PID error'); hold on;
plot(t_smc, rad2deg(e_chi_smc),'Color',c_smc,'LineWidth',lw,'DisplayName','SMC error');
yline(0,'-k','LineWidth',0.5);
yline(5,'--','Color',c_ref,'LineWidth',0.8,'Label','±5° band');
yline(-5,'--','Color',c_ref,'LineWidth',0.8);
xlabel('t (s)','FontSize',fs); ylabel('e_χ (deg)','FontSize',fs);
title('Heading error','FontSize',fs); legend('Location','northeast','FontSize',9); grid on;

subplot(1,3,3)
plot(t_smc, s_psi_smc,'Color',c_smc,'LineWidth',lw,'DisplayName','s_{ψ}');
yline(0,'-k','LineWidth',0.5);
yline(0.1,'--','Color',c_ref,'LineWidth',0.8,'Label','Boundary layer φ');
yline(-0.1,'--','Color',c_ref,'LineWidth',0.8);
xlabel('t (s)','FontSize',fs); ylabel('s_ψ','FontSize',fs);
title('SMC sliding surface','FontSize',fs); legend('Location','northeast','FontSize',9); grid on;

sgtitle('Phase 9: PID vs SMC — heading step 0→45°, u=1.5 m/s','FontSize',fs);

% =========================================================================
% Figure 2 — Architecture validation: how to swap controllers
% =========================================================================
fprintf('\n=== HOW TO SWAP CONTROLLERS IN run_simulation.m ===\n\n');
fprintf('Current (PID):\n');
fprintf('  cs = ctrl_init_local(auv_sim);       %% Phase 5\n');
fprintf('  ...\n');
fprintf('  [tau_ctrl, n_direct, ctrl_dbg, cs] = ctrl_step_local(...);\n\n');
fprintf('To use SMC — change ONLY these two lines:\n');
fprintf('  cs = control_smc_init_local(auv_sim);   %% Phase 9\n');
fprintf('  ...\n');
fprintf('  [tau_ctrl, n_direct, ctrl_dbg, cs] = control_smc_step_local(...);\n\n');
fprintf('No other module changes. This is the plug-and-play contract.\n\n');

% =========================================================================
% Summary statistics
% =========================================================================
fprintf('=== Performance Summary ===\n');
k_pid = numel(t_pid);
k_smc = numel(t_smc);

rms_pid = sqrt(mean(e_chi_pid(round(end/3):end).^2));
rms_smc = sqrt(mean(e_chi_smc(round(end/3):end).^2));

psi_pid_max = max(rad2deg(X_pid(:,12)));
psi_smc_max = max(rad2deg(X_smc(:,12)));

fprintf('  %-14s  %10s  %12s  %14s\n','Controller','RMS error','Final psi','Max overshoot');
fprintf('  %-14s  %8.2f deg  %10.1f deg  %12.1f deg\n', ...
    'PID', rad2deg(rms_pid), rad2deg(X_pid(end,12)), max(0,psi_pid_max-45));
fprintf('  %-14s  %8.2f deg  %10.1f deg  %12.1f deg\n', ...
    'SMC', rad2deg(rms_smc), rad2deg(X_smc(end,12)), max(0,psi_smc_max-45));
fprintf('\nSMC characteristics:\n');
fprintf('  Boundary layer phi = 0.10 rad\n');
fprintf('  Surface slope lambda = 3.0 rad/s\n');
fprintf('  Reaching gain k = 2.0\n');
fprintf('  Expected steady-state band: phi*k/lambda = %.2f rad = %.1f deg\n\n', ...
    0.1*2.0/3.0, rad2deg(0.1*2.0/3.0));
