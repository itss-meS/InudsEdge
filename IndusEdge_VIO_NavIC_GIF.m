%% IndusEdge_VIO_NavIC_GIF.m
% Side-by-side animated GIF: trajectory fusion plot (left) + KITTI camera frame (right).
% Uses REAL output from run_eskf.m, time-aligned to ground-truth (10 Hz) frames.
% Ground Truth is drawn in full from frame 1 (known upfront); VIO/NavIC/EKF grow over time.
% Run run_eskf.m first.

clear; clc; close all;

%% --- Paths (EDIT THESE) ---
poses_file        = 'poses/00.txt';
predict_only_file = 'predict_only_traj.txt';
predict_only_t    = 'predict_only_traj_t.txt';
eskf_file         = 'eskf_traj.txt';
eskf_t_file       = 'eskf_traj_t.txt';
navic_file        = 'NavIC_data/fake_navic_00.csv';
image_folder      = 'D:/2k26-27/SIH/Camera_dataset/dataset/sequences/00/image_0';   % <-- set this
gif_filename      = 'IndusEdge_VIO_NavIC.gif';
gif_frame_step    = 5;

%% --- Ground truth + timestamps (10 Hz reference) ---
gt = readmatrix(poses_file);
ground_truth_path = [gt(:,4), gt(:,12)];
N = size(ground_truth_path, 1);
gt_t = (0:N-1)' * 0.1;

%% --- Align IMU-rate trajectories onto ground-truth timestamps ---
pos_vio = align_traj_to_gt(predict_only_file, predict_only_t, gt_t);
x_est   = align_traj_to_gt(eskf_file, eskf_t_file, gt_t)';   % 2xN to match plotting below

%% --- NavIC visibility epochs (marks where an update occurred) ---
navic = readtable(navic_file);
vis_raw = navic.visible;
if iscell(vis_raw) || isstring(vis_raw) || iscategorical(vis_raw)
    navic_vis = ismember(lower(string(vis_raw)), ["true", "1", "yes"]);
else
    navic_vis = logical(vis_raw);
end
navic_vis_frames = unique(navic.frame(navic_vis));
navic_vis_frames = navic_vis_frames(navic_vis_frames >= 1 & navic_vis_frames <= N);
z_gnss = ground_truth_path(navic_vis_frames, :)';
gnss_indices = navic_vis_frames';

%% --- Fixed axis limits (based on Ground Truth ONLY) ---
% Deliberately do NOT fold x_est (or pos_vio) into this range. If the
% estimate drifts even a little, min/max over a divergent trajectory
% dwarfs the real GT route and the whole plot collapses to a speck at a
% 10^4+ m scale. Locking the camera to the true route (like the reference
% VIO+GNSS plot) keeps the picture meaningful; a diverging EKF/VIO trace
% will simply run off the edge of frame, which is itself informative.
all_x = ground_truth_path(:,1);
all_z = ground_truth_path(:,2);
range_x = range(all_x);
range_z = range(all_z);
margin = 0.08 * max(range_x, range_z);
if margin == 0
    margin = 5;
end
xlims = [min(all_x)-margin, max(all_x)+margin];
zlims = [min(all_z)-margin, max(all_z)+margin];

fprintf('GT route extent: X [%.1f, %.1f] m, Z [%.1f, %.1f] m\n', ...
    min(all_x), max(all_x), min(all_z), max(all_z));
final_err = sqrt(sum((x_est(:,end) - ground_truth_path(end,:)').^2));
fprintf('Final EKF position error: %.2f m (final GT point vs final EKF point)\n', final_err);
if final_err > 3 * max(range_x, range_z)
    fprintf(['WARNING: EKF has drifted far outside the GT route extent. ', ...
        'The estimate will run off-frame in the GIF -- this is a filter ', ...
        'divergence issue, not a plotting issue.\n']);
end

%% --- Animate + write GIF ---
image_files = dir(fullfile(image_folder, '*.png'));
fig = figure('Position', [100, 100, 1400, 600]);

for k = 2:gif_frame_step:N
    clf(fig);

    subplot(1,2,1);
    hold on; grid on;
    plot(ground_truth_path(:,1), ground_truth_path(:,2), 'k-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
    plot(pos_vio(1:k,1), pos_vio(1:k,2), 'r--', 'LineWidth', 1.2, 'DisplayName', 'VIO (IMU-only)');

    gnss_to_plot = gnss_indices(gnss_indices <= k);
    if ~isempty(gnss_to_plot)
        plot(z_gnss(1, 1:length(gnss_to_plot)), z_gnss(2, 1:length(gnss_to_plot)), ...
            'bo', 'MarkerSize', 5, 'DisplayName', 'NavIC');
    end

    plot(x_est(1,1:k), x_est(2,1:k), 'g-', 'LineWidth', 2, 'DisplayName', 'EKF');
    xlim(xlims); ylim(zlims); axis equal;
    xlabel('X (m)'); ylabel('Z (m)'); title('IndusEdge ESKF: VIO + NavIC Fusion');
    legend('Location', 'best');

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

fprintf('Saved animation to %s\n', gif_filename);