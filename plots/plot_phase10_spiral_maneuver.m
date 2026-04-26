%% plot_phase10_spiral_maneuver.m — Visualizing the Spiral + Torpedo Maneuver Test

if ~exist('phase10_spiral_maneuver_results.mat','file')
    error('Run test_phase10_spiral_maneuver first.');
end
load('phase10_spiral_maneuver_results.mat');

c_pid = [0.13 0.47 0.71];  % Blue
c_smc = [0.84 0.15 0.16];  % Red
c_ref = [0.25 0.25 0.25];  % Dark grey
c_sw  = [0.20 0.55 0.20];  % Green
lw = 1.6;
fs = 11;

close all;

% Common metrics
chi_ref_deg = rad2deg(ref_pid(:,4));
psi_pid_deg = rad2deg(unwrap(X_pid(:,12)));
psi_smc_deg = rad2deg(unwrap(X_smc(:,12)));

rud_pid = rad2deg(ui_pid(1,:));
rud_smc = rad2deg(ui_smc(1,:));
sp_pid  = rad2deg(ui_pid(2,:));
sp_smc  = rad2deg(ui_smc(2,:));

% -------------------------------------------------------------------------
% Figure 1: 3D trajectory
% -------------------------------------------------------------------------
figure(1); set(gcf,'Position',[80 80 750 580],'Name','3D Trajectory');
plot3(ref_pid(:,2), ref_pid(:,1), ref_pid(:,3), '--', 'Color', c_ref, 'LineWidth', 2.0, 'DisplayName', 'Reference'); hold on;
plot3(X_pid(:,8),   X_pid(:,7),   X_pid(:,9),   'Color', c_pid, 'LineWidth', lw, 'DisplayName', 'PID');
plot3(X_smc(:,8),   X_smc(:,7),   X_smc(:,9),   'Color', c_smc, 'LineWidth', lw, 'DisplayName', 'SMC');
plot3(ref_pid(find(phase_pid==2,1,'first'),2), ref_pid(find(phase_pid==2,1,'first'),1), ref_pid(find(phase_pid==2,1,'first'),3), ...
      'o', 'Color', c_sw, 'MarkerFaceColor', c_sw, 'DisplayName', 'Maneuver Start');
grid on; axis equal;
set(gca,'ZDir','reverse');
xlabel('East (y_E) [m]', 'FontSize', fs);
ylabel('North (x_N) [m]', 'FontSize', fs);
zlabel('Depth [m]', 'FontSize', fs);
title('3D Reference Tracking: Spiral Search to Terminal Maneuver', 'FontSize', fs+2);
legend('Location','best');
view(42,26);

% -------------------------------------------------------------------------
% Figure 2: Top-down plan view
% -------------------------------------------------------------------------
figure(2); set(gcf,'Position',[870 80 700 560],'Name','Top-Down Plan View');
plot(ref_pid(:,2), ref_pid(:,1), '--', 'Color', c_ref, 'LineWidth', 2.0, 'DisplayName', 'Reference'); hold on;
plot(X_pid(:,8),   X_pid(:,7),   'Color', c_pid, 'LineWidth', lw, 'DisplayName', 'PID Path');
plot(X_smc(:,8),   X_smc(:,7),   'Color', c_smc, 'LineWidth', lw, 'DisplayName', 'SMC Path');
idx_sw = find(phase_pid==2,1,'first');
plot(ref_pid(idx_sw,2), ref_pid(idx_sw,1), 'o', 'Color', c_sw, 'MarkerFaceColor', c_sw, 'DisplayName', 'Phase Switch');
text(ref_pid(idx_sw,2), ref_pid(idx_sw,1), '  Terminal maneuver', 'Color', c_sw, 'FontSize', fs);
grid on; axis equal;
xlabel('East (y_E) [m]', 'FontSize', fs);
ylabel('North (x_N) [m]', 'FontSize', fs);
title('Plan View Comparison', 'FontSize', fs+2);
legend('Location','best');

% -------------------------------------------------------------------------
% Figure 3: Cross-track and depth errors
% -------------------------------------------------------------------------
figure(3); set(gcf,'Position',[80 700 760 520],'Name','Tracking Errors');
subplot(2,1,1);
plot(t_pid, err_pid(:,2), 'Color', c_pid, 'LineWidth', lw, 'DisplayName', 'PID y_e'); hold on;
plot(t_smc, err_smc(:,2), 'Color', c_smc, 'LineWidth', lw, 'DisplayName', 'SMC y_e');
xline(T_switch, '--', 'Color', c_sw, 'LineWidth', 1.5, 'DisplayName', 'Phase Switch');
yline(0, '--k', 'HandleVisibility','off');
grid on;
xlabel('Time [s]', 'FontSize', fs);
ylabel('Cross-track error [m]', 'FontSize', fs);
title('Lateral Tracking Error', 'FontSize', fs+2);
legend('Location','best');

subplot(2,1,2);
plot(t_pid, err_pid(:,3), 'Color', c_pid, 'LineWidth', lw, 'DisplayName', 'PID z_e'); hold on;
plot(t_smc, err_smc(:,3), 'Color', c_smc, 'LineWidth', lw, 'DisplayName', 'SMC z_e');
xline(T_switch, '--', 'Color', c_sw, 'LineWidth', 1.5, 'DisplayName', 'Phase Switch');
yline(0, '--k', 'HandleVisibility','off');
grid on;
xlabel('Time [s]', 'FontSize', fs);
ylabel('Depth error [m]', 'FontSize', fs);
title('Depth Tracking Error', 'FontSize', fs+2);
legend('Location','best');

% -------------------------------------------------------------------------
% Figure 4: Heading tracking
% -------------------------------------------------------------------------
figure(4); set(gcf,'Position',[870 700 760 520],'Name','Heading Response');
plot(t_pid, chi_ref_deg, '--', 'Color', c_ref, 'LineWidth', 2.0, 'DisplayName', 'Reference heading'); hold on;
plot(t_pid, psi_pid_deg, 'Color', c_pid, 'LineWidth', lw, 'DisplayName', 'PID \psi');
plot(t_smc, psi_smc_deg, 'Color', c_smc, 'LineWidth', lw, 'DisplayName', 'SMC \psi');
xline(T_switch, '--', 'Color', c_sw, 'LineWidth', 1.5, 'DisplayName', 'Phase Switch');
grid on;
xlabel('Time [s]', 'FontSize', fs);
ylabel('Heading [deg]', 'FontSize', fs);
title('Heading Alignment During Spiral and Terminal Weave', 'FontSize', fs+2);
legend('Location','best');

% -------------------------------------------------------------------------
% Figure 5: Actuator effort
% -------------------------------------------------------------------------
figure(5); set(gcf,'Position',[1640 80 760 560],'Name','Actuator Effort');
subplot(3,1,1);
plot(t_pid, rud_pid, 'Color', c_pid, 'LineWidth', lw, 'DisplayName', 'PID rudder'); hold on;
plot(t_smc, rud_smc, 'Color', c_smc, 'LineWidth', lw, 'DisplayName', 'SMC rudder');
xline(T_switch, '--', 'Color', c_sw, 'LineWidth', 1.5, 'DisplayName', 'Phase Switch');
yline(20, '--r', 'HandleVisibility','off'); yline(-20, '--r', 'HandleVisibility','off');
grid on;
ylabel('\delta_r [deg]', 'FontSize', fs);
title('Rudder Demand', 'FontSize', fs+2);
legend('Location','best');

subplot(3,1,2);
plot(t_pid, sp_pid, 'Color', c_pid, 'LineWidth', lw, 'DisplayName', 'PID stern plane'); hold on;
plot(t_smc, sp_smc, 'Color', c_smc, 'LineWidth', lw, 'DisplayName', 'SMC stern plane');
xline(T_switch, '--', 'Color', c_sw, 'LineWidth', 1.5, 'DisplayName', 'Phase Switch');
yline(20, '--r', 'HandleVisibility','off'); yline(-20, '--r', 'HandleVisibility','off');
grid on;
ylabel('\delta_s [deg]', 'FontSize', fs);
title('Stern Plane Demand', 'FontSize', fs+2);
legend('Location','best');

subplot(3,1,3);
plot(t_pid, ui_pid(3,:), 'Color', c_pid, 'LineWidth', lw, 'DisplayName', 'PID prop'); hold on;
plot(t_smc, ui_smc(3,:), 'Color', c_smc, 'LineWidth', lw, 'DisplayName', 'SMC prop');
xline(T_switch, '--', 'Color', c_sw, 'LineWidth', 1.5, 'DisplayName', 'Phase Switch');
grid on;
xlabel('Time [s]', 'FontSize', fs);
ylabel('n_d', 'FontSize', fs);
title('Propulsor Command', 'FontSize', fs+2);
legend('Location','best');

% -------------------------------------------------------------------------
% Figure 6: Summary bars
% -------------------------------------------------------------------------
rms_y_pid = sqrt(mean(err_pid(:,2).^2));
rms_y_smc = sqrt(mean(err_smc(:,2).^2));
rms_z_pid = sqrt(mean(err_pid(:,3).^2));
rms_z_smc = sqrt(mean(err_smc(:,3).^2));
peak_r_pid = max(abs(rud_pid));
peak_r_smc = max(abs(rud_smc));
final_p_pid = norm(X_pid(end,7:9) - ref_pid(end,1:3));
final_p_smc = norm(X_smc(end,7:9) - ref_smc(end,1:3));

figure(6); set(gcf,'Position',[1640 700 700 500],'Name','Performance Summary');
vals = [rms_y_pid rms_y_smc; rms_z_pid rms_z_smc; peak_r_pid peak_r_smc; final_p_pid final_p_smc];
b = bar(vals); grid on;
set(gca,'XTickLabel',{'RMS y_e [m]','RMS z_e [m]','Peak |\delta_r| [deg]','Final pos err [m]'}, 'FontSize', fs);
b(1).FaceColor = c_pid; b(2).FaceColor = c_smc;
legend({'PID','SMC'}, 'Location', 'northwest');
title('Performance Summary', 'FontSize', fs+2);

% Console summary
fprintf('\n--- PHASE 10: SPIRAL + TORPEDO MANEUVER RESULTS ---\n');
fprintf('RMS Cross-Track Error y_e:\n');
fprintf('  PID: %.2f m\n', rms_y_pid);
fprintf('  SMC: %.2f m\n', rms_y_smc);
fprintf('RMS Depth Error z_e:\n');
fprintf('  PID: %.2f m\n', rms_z_pid);
fprintf('  SMC: %.2f m\n', rms_z_smc);
fprintf('Peak Rudder Demand:\n');
fprintf('  PID: %.2f deg\n', peak_r_pid);
fprintf('  SMC: %.2f deg\n', peak_r_smc);
fprintf('Final Position Error:\n');
fprintf('  PID: %.2f m\n', final_p_pid);
fprintf('  SMC: %.2f m\n', final_p_smc);
