%% plot_phase5.m  —  Phase 5 Diagnostic Plots
%
% Generates four figures from the closed-loop step response data
% saved by test_phase5_control.m.
%
% HOW TO RUN:
%   >> plot_phase5

if ~exist('phase5_results.mat','file')
    error('Run test_phase5_control first.');
end
load('phase5_results.mat');

lw = 1.5; fs = 11;
c_actual = [0.13 0.47 0.71];
c_ref    = [0.84 0.15 0.16];
c_aux    = [0.17 0.63 0.17];
close all;

% =========================================================================
% Figure 1 — Surge step response
% =========================================================================
figure(1);
set(gcf,'Name','Phase 5 — Surge step response','NumberTitle','off','Position',[50 500 650 320]);
plot(results.t1, results.X1(:,1), 'Color',c_actual,'LineWidth',lw); hold on;
yline(1.5,'--','Color',c_ref,'LineWidth',1,'Label','Reference 1.5 m/s');
yline(1.4,':','Color',c_aux,'LineWidth',1,'Label','±0.1 band');
yline(1.6,':','Color',c_aux,'LineWidth',1);
xlabel('Time (s)','FontSize',fs); ylabel('u  (m/s)','FontSize',fs);
title('Surge: step 0 → 1.5 m/s','FontSize',fs); grid on; xlim([0 60]); ylim([-0.1 2]);

% =========================================================================
% Figure 2 — Depth step response
% =========================================================================
figure(2);
set(gcf,'Name','Phase 5 — Depth step response','NumberTitle','off','Position',[50 100 900 500]);

subplot(2,2,1)
plot(results.t2, results.X2(:,9), 'Color',c_actual,'LineWidth',lw); hold on;
yline(results.z_des,'--','Color',c_ref,'LineWidth',1,'Label',sprintf('%.0fm target',results.z_des));
xlabel('t (s)','FontSize',fs); ylabel('z_D (m)  +ve=down','FontSize',fs);
title('Depth','FontSize',fs); grid on; set(gca,'YDir','reverse');

subplot(2,2,2)
plot(results.t2, rad2deg(results.X2(:,11)), 'Color',c_aux,'LineWidth',lw); hold on;
yline(0,'-k','LineWidth',0.5);
xlabel('t (s)','FontSize',fs); ylabel('theta (deg)','FontSize',fs);
title('Pitch angle','FontSize',fs); grid on;

subplot(2,2,3)
plot(results.t2, results.X2(:,1), 'Color',c_actual,'LineWidth',lw);
xlabel('t (s)','FontSize',fs); ylabel('u (m/s)','FontSize',fs);
title('Surge speed during dive','FontSize',fs); grid on;

subplot(2,2,4)
plot(results.t2, results.X2(:,3), 'Color',c_actual,'LineWidth',lw); hold on;
yline(0,'-k','LineWidth',0.5);
xlabel('t (s)','FontSize',fs); ylabel('w (m/s)','FontSize',fs);
title('Heave velocity','FontSize',fs); grid on;

sgtitle('Phase 5: Depth step 0 → 5m at u=1.5 m/s','FontSize',fs);

% =========================================================================
% Figure 3 — Heading step 0 → 45 deg
% =========================================================================
figure(3);
set(gcf,'Name','Phase 5 — Heading step response','NumberTitle','off','Position',[710 500 650 320]);
plot(results.t3, rad2deg(results.X3(:,12)), 'Color',c_actual,'LineWidth',lw); hold on;
yline(rad2deg(results.psi_des),'--','Color',c_ref,'LineWidth',1,'Label','45 deg target');
yline(rad2deg(results.psi_des)+3,':','Color',c_aux,'LineWidth',1,'Label','±3 deg band');
yline(rad2deg(results.psi_des)-3,':','Color',c_aux,'LineWidth',1);
xlabel('Time (s)','FontSize',fs); ylabel('psi (deg)','FontSize',fs);
title('Heading: step 0 → 45 deg','FontSize',fs); grid on; xlim([0 40]);

% =========================================================================
% Figure 4 — Heading across ±pi boundary
% =========================================================================
figure(4);
set(gcf,'Name','Phase 5 — Boundary crossing test','NumberTitle','off','Position',[710 100 650 320]);
plot(results.t4, rad2deg(results.X4(:,12)), 'Color',c_actual,'LineWidth',lw); hold on;
yline(-170,'--','Color',c_ref,'LineWidth',1,'Label','-170 deg target');
yline(0,'-k','LineWidth',0.5,'Label','0 deg start');
xlabel('Time (s)','FontSize',fs); ylabel('psi (deg)','FontSize',fs);
title('Heading boundary test: 0 → -170 deg (short-way turn)','FontSize',fs);
grid on; xlim([0 60]);

fprintf('\nPhase 5 plots generated (Figures 1-4).\n');
fprintf('Expected observations:\n');
fprintf('  Fig 1: Smooth S-curve to 1.5 m/s, no oscillation\n');
fprintf('  Fig 2: Depth reaches 5m, pitch stays within ±25 deg\n');
fprintf('  Fig 3: Heading reaches 45 deg in < 30s, small overshoot\n');
fprintf('  Fig 4: Turns LEFT (negative psi), never goes positive\n\n');
fprintf('If any response looks wrong → retune auv.ctrl gains in auv_params.m\n');
fprintf('and re-run the gate test. No other files need changes.\n\n');
