%% IndusEdge_GT_vs_EKF_GIF.m
% Side-by-side animated GIF comparing EKF estimate against ground truth (GT hidden).
% Left: EKF (grows over time) + live error/RMSE vs Hidden Ground Truth.
% Right: corresponding KITTI camera frame.
% Run run_eskf.m first.

clear; clc; close all;

%% --- Paths (EDIT THESE) ---
poses_file     = 'poses/00.txt';
eskf_file      = 'eskf_traj.txt';
eskf_t_file    = 'eskf_traj_t.txt';
image_folder   = 'D:/2k26-27/SIH/Camera_dataset/dataset/sequences/00/image_1';   % <-- set this
gif_filename   = 'IndusEdge_GT_vs_EKF.gif';
gif_frame_step = 5;

%% --- Ground truth + timestamps ---
gt = readmatrix(poses_file);
ground_truth_path = [gt(:,4), gt(:,12)];
N = size(ground_truth_path, 1);
gt_t = (0:N-1)' * 0.1;

%% --- EKF estimate, time-aligned to ground truth ---
est_path = align_traj_to_gt(eskf_file, eskf_t_file, gt_t);   % [Nx2]

%% --- Per-frame error ---
err = sqrt(sum((est_path - ground_truth_path).^2, 2));   % [Nx1] position error, meters

%% --- Fixed axis limits ---
% We keep calculating limits based on GT so the view stays stable 
% even though the black line is hidden.
all_x = ground_truth_path(:,1);
all_z = ground_truth_path(:,2);
range_x = range(all_x);
range_z = range(all_z);
margin = 0.08 * max(range_x, range_z);
if margin == 0
    margin = 5;  % degenerate case: stationary/near-zero GT extent
end
xlims = [min(all_x)-margin, max(all_x)+margin];
zlims = [min(all_z)-margin, max(all_z)+margin];

fprintf('GT route extent: X [%.1f, %.1f] m, Z [%.1f, %.1f] m\n', ...
    min(all_x), max(all_x), min(all_z), max(all_z));
fprintf('Final position error: %.2f m | Final running RMSE: %.2f m\n', ...
    err(end), sqrt(mean(err.^2)));

%% --- Animate + write GIF ---
image_files = dir(fullfile(image_folder, '*.png'));
fig = figure('Position', [100, 100, 1400, 600]);

for k = 2:gif_frame_step:N
    clf(fig);

    subplot(1,2,1);
    hold on; grid on;
    
    % --- GROUND TRUTH PLOT REMOVED/COMMENTED BELOW ---
    % plot(ground_truth_path(:,1), ground_truth_path(:,2), 'k-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
    
    % Plot EKF trajectory
    plot(est_path(1:k,1), est_path(1:k,2), 'g-', 'LineWidth', 2, 'DisplayName', 'EKF');
    
    % Plot live EKF head (moving dot)
    plot(est_path(k,1), est_path(k,2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 5, ...
        'HandleVisibility', 'off');
    
    % Plot error connector (magenta line) - points from EKF to the invisible GT point
    plot([ground_truth_path(k,1), est_path(k,1)], [ground_truth_path(k,2), est_path(k,2)], ...
        'm:', 'LineWidth', 1, 'HandleVisibility', 'off');   

    xlim(xlims); ylim(zlims); axis equal;
    xlabel('X (m)'); ylabel('Z (m)'); title('EKF Estimate');
    legend('Location', 'best');

    % Display error text
    current_err = err(k);
    running_rmse = sqrt(mean(err(1:k).^2));
    text(0.02, 0.98, sprintf('Error: %.2f m\nRunning RMSE: %.2f m', current_err, running_rmse), ...
        'Units', 'normalized', 'VerticalAlignment', 'top', 'BackgroundColor', 'w');

    subplot(1,2,2);
    if k <= length(image_files)
        img_path = fullfile(image_folder, image_files(k).name);
        if isfile(img_path)
            imshow(imread(img_path));
        end
    end
    title(sprintf('Live Camera View (Frame %d)', k));

    drawnow;
    frame = getframe(fig);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    if k == 2
        imwrite(imind, cm, gif_filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.05);
    else
        imwrite(imind, cm, gif_filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
    end
end

fprintf('Final RMSE: %.3f m\n', sqrt(mean(err.^2)));
fprintf('Saved animation to %s\n', gif_filename);