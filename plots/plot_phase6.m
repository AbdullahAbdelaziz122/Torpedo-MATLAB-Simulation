%% plot_phase6.m  —  Phase 6 Diagnostic Plots
%
% HOW TO RUN:
%   >> plot_phase6

if ~exist('phase6_results.mat','file')
    error('Run test_phase6_guidance first.');
end
load('phase6_results.mat');

lw=1.5; fs=11;
c_path   = [0.5 0.5 0.5];
c_vehicle= [0.13 0.47 0.71];
c_err    = [0.84 0.15 0.16];
close all;

% =========================================================================
% Figure 1 — 3D trajectory
% =========================================================================
figure(1);
set(gcf,'Name','Phase 6 — 3D path following','NumberTitle','off','Position',[50 400 700 500]);
plot3(path_test.pts(1,:), path_test.pts(2,:), -path_test.pts(3,:), ...
    '--','Color',c_path,'LineWidth',1.5,'DisplayName','Reference helix'); hold on;
plot3(X_cl(:,7), X_cl(:,8), -X_cl(:,9), ...
    'Color',c_vehicle,'LineWidth',lw,'DisplayName','Vehicle trajectory');
scatter3(X_cl(1,7), X_cl(1,8), -X_cl(1,9), 60,'g','filled','DisplayName','Start');
scatter3(X_cl(end,7), X_cl(end,8), -X_cl(end,9), 60,'r','filled','DisplayName','End');
xlabel('x_N (m)','FontSize',fs); ylabel('y_E (m)','FontSize',fs);
zlabel('Altitude (m, up=positive)','FontSize',fs);
title('3D Path Following — LOS Guidance','FontSize',fs);
legend('Location','best','FontSize',9); grid on; 
daspect([1 1 0.05]); 
axis equal;

view(30, 25);

% =========================================================================
% Figure 2 — Cross-track errors over time
% =========================================================================
figure(2);
set(gcf,'Name','Phase 6 — Cross-track errors','NumberTitle','off','Position',[760 400 700 450]);

subplot(3,1,1)
plot(t_cl, errs_cl(:,1),'Color',c_vehicle,'LineWidth',lw);
yline(0,'-k','LineWidth',0.5);
xlabel('t (s)','FontSize',fs); ylabel('x_e (m)','FontSize',fs);
title('Along-track error  (+ = behind path point)','FontSize',fs); grid on;

subplot(3,1,2)
plot(t_cl, errs_cl(:,2),'Color',c_err,'LineWidth',lw);
yline(0,'-k','LineWidth',0.5);
xlabel('t (s)','FontSize',fs); ylabel('y_e (m)','FontSize',fs);
title('Lateral cross-track error  (+ = right of path)','FontSize',fs); grid on;

subplot(3,1,3)
plot(t_cl, errs_cl(:,3),'Color',[0.17 0.63 0.17],'LineWidth',lw);
yline(0,'-k','LineWidth',0.5);
xlabel('t (s)','FontSize',fs); ylabel('z_e (m)','FontSize',fs);
title('Vertical cross-track error  (+ = below path)','FontSize',fs); grid on;

sgtitle('Phase 6: LOS Cross-Track Errors','FontSize',fs);

% =========================================================================
% Figure 3 — Speed and depth
% =========================================================================
figure(3);
set(gcf,'Name','Phase 6 — Speed and depth','NumberTitle','off','Position',[50 50 700 400]);

subplot(2,1,1)
plot(t_cl, X_cl(:,1),'Color',c_vehicle,'LineWidth',lw); hold on;
yline(1.5,'--','Color',c_path,'LineWidth',1,'Label','1.5 m/s ref');
xlabel('t (s)','FontSize',fs); ylabel('u (m/s)','FontSize',fs);
title('Surge speed','FontSize',fs); grid on;

subplot(2,1,2)
plot(t_cl, X_cl(:,9),'Color',c_err,'LineWidth',lw); hold on;
plot(t_cl, path_test.pts(3, 1:min(numel(t_cl),size(path_test.pts,2))), ...
    '--','Color',c_path,'LineWidth',1,'DisplayName','Path depth');
xlabel('t (s)','FontSize',fs); ylabel('z_D (m)  +ve=down','FontSize',fs);
title('Depth tracking','FontSize',fs); grid on; set(gca,'YDir','reverse');
legend('Vehicle','Path','Location','best','FontSize',9);

fprintf('\nPhase 6 plots generated (Figures 1-3).\n');
fprintf('Expected observations:\n');
fprintf('  Fig 1: Vehicle spiral tracks grey reference helix\n');
fprintf('  Fig 2: y_e and z_e decay toward 0 after initial transient\n');
fprintf('  Fig 3: Speed converges to ~1.5 m/s, depth tracks path\n\n');
