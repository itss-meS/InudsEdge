function xz_aligned = align_traj_to_gt(traj_file, time_file, gt_t)
% Interpolates a KITTI-format trajectory (saved at IMU rate) onto the
% ground-truth timestamps gt_t, returning an [Nx2] array of [X, Z].

    traj = readmatrix(traj_file);
    t    = readmatrix(time_file);

    xz = [traj(:,4), traj(:,12)];

    x_i = interp1(t, xz(:,1), gt_t, 'linear', 'extrap');
    z_i = interp1(t, xz(:,2), gt_t, 'linear', 'extrap');

    xz_aligned = [x_i, z_i];
end
