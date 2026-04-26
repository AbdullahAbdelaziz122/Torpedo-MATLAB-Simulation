%% plot_phase9_maneuver.m — Visualizing the Maneuver Stress Test

if ~exist('phase9_maneuver_results.mat','file')
    error('Run test_phase9_maneuver first.');
end
load('phase9_maneuver_results.mat');

c_pid = [0.13 0.47 0.71]; % Blue
c_smc = [0.84 0.15 0.16]; % Red
c_ref = [0.5 0.5 0.5];    % Grey
lw = 1.5; fs = 11;
close all;

% --- Figure 1: 3D Trajectory ---
figure(1); set(gcf,'Position',[50 100 700 600],'Name','3D Corkscrew Trajectory');
plot3(X_pid(:,8), X_pid(:,7), -X_pid(:,9), 'Color', c_pid, 'LineWidth', lw, 'DisplayName', 'PID AUV'); hold on;
plot3(X_smc(:,8), X_smc(:,7), -X_smc(:,9), 'Color', c_smc, 'LineWidth', lw, 'DisplayName', 'SMC AUV');
plot3(pt_hist(2,:), pt_hist(1,:), -pt_hist(3,:), '--', 'Color', c_ref, 'LineWidth', 1.5, 'DisplayName', 'Target Path');
scatter3(pt_hist(2,1), pt_hist(1,1), -pt_hist(3,1), 100, 'g', 'filled', 'DisplayName', 'Start');
scatter3(pt_hist(2,end), pt_hist(1,end), -pt_hist(3,end), 100, 'r', 'filled', 'DisplayName', 'End');
xlabel('East (y_E) [m]', 'FontSize', fs); ylabel('North (x_N) [m]', 'FontSize', fs); zlabel('Altitude (Up +ve) [m]', 'FontSize', fs);
title('High-Speed Corkscrew Dive & Intercept', 'FontSize', fs+2);
legend('Location','best'); grid on; daspect([1 1 0.2]); view(35, 30);

% --- Figure 2: Depth Tracking (Wind-up Check) ---
figure(2); set(gcf,'Position',[800 500 600 400],'Name','Depth Tracking');
plot(t_pid, X_pid(:,9), 'Color', c_pid, 'LineWidth', lw, 'DisplayName', 'PID Depth (z_D)'); hold on;
plot(t_smc, X_smc(:,9), 'Color', c_smc, 'LineWidth', lw, 'DisplayName', 'SMC Depth (z_D)');
plot(t_pid, pt_hist(3,:), '--k', 'LineWidth', 1.5, 'DisplayName', 'Target Depth');
xline(10, ':', 'Color', c_ref, 'HandleVisibility','off'); 
xline(40, ':', 'Color', c_ref, 'HandleVisibility','off');
set(gca, 'YDir', 'reverse'); % Depth increases downwards
xlabel('Time [s]', 'FontSize', fs); ylabel('Depth [m]', 'FontSize', fs);
title('Depth Control: Dive and Sharp Level-Off', 'FontSize', fs+2);
legend('Location','best'); grid on;

% --- Figure 3: Stern Plane Actuation (Pitch Control) ---
figure(3); set(gcf,'Position',[800 50 600 400],'Name','Stern Plane Actuation');
plot(t_pid, rad2deg(ui_pid(2,:)), 'Color', c_pid, 'LineWidth', lw, 'DisplayName', 'PID Stern Plane (\delta_s)'); hold on;
plot(t_smc, rad2deg(ui_smc(2,:)), 'Color', c_smc, 'LineWidth', lw, 'DisplayName', 'SMC Stern Plane (\delta_s)');
yline(20, '--r', 'LineWidth', 1, 'HandleVisibility','off');
yline(-20, '--r', 'LineWidth', 1, 'HandleVisibility','off');
xlabel('Time [s]', 'FontSize', fs); ylabel('Deflection [deg]', 'FontSize', fs);
title('Stern Plane Effort (Pitch & Heave control)', 'FontSize', fs+2);
legend('Location','best'); grid on;

% Quantitative output
z_err_pid = sqrt(mean((X_pid(:,9)' - pt_hist(3,:)).^2));
z_err_smc = sqrt(mean((X_smc(:,9)' - pt_hist(3,:)).^2));
fprintf('\n--- MANEUVER TEST RESULTS ---\n');
fprintf('RMS Depth Error during maneuver:\n');
fprintf('  PID: %.2f meters\n', z_err_pid);
fprintf('  SMC: %.2f meters\n', z_err_smc);