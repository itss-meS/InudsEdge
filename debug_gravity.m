% debug_gravity.m
fprintf('\n========== GRAVITY FRAME DEBUGGER ==========\n');
if ~exist('IMU_data/fake_imu_00.csv', 'file')
    error('IMU file not found. Check path.');
end
imu_raw = readmatrix('IMU_data/fake_imu_00.csv');
raw_acc = imu_raw(:,2:4); raw_gyr = imu_raw(:,5:7);

static_idx = 1:min(200, size(imu_raw,1));
mean_acc = mean(raw_acc(static_idx, :), 1)';
mean_gyr = mean(raw_gyr(static_idx, :), 1)';

fprintf('Raw CSV Mean Acc: [%.4f, %.4f, %.4f] (Mag: %.4f)\n', mean_acc', norm(mean_acc));
fprintf('Raw CSV Mean Gyr: [%.6f, %.6f, %.6f]\n', mean_gyr');

[~, max_idx] = max(abs(mean_acc));
val = mean_acc(max_idx);

if val > 0
    sign_str = 'POSITIVE'; map_str = 'FLIP (Multiply by -1)';
else
    sign_str = 'NEGATIVE'; map_str = 'KEEP (Multiply by +1)';
end

fprintf('\n>>> GRAVITY ON CSV AXIS %d | VALUE: %.4f (%s)\n', max_idx, val, sign_str);
fprintf('\n--- MAPPING TO KITTI FRD (Y-Down) ---\n');
fprintf('Map CSV Axis %d -> FRD Y. %s\n', max_idx, map_str);

fprintf('\n>>> SUGGESTED load_data MAPPING:\n');
switch max_idx
    case 1, fprintf('%% Gravity on X? Check mount.\n');
    case 2
        if val > 0 % Y-Down (FRD Native)
            fprintf('obj.imu_acc  = [ raw_acc(:,1),  raw_acc(:,2), -raw_acc(:,3) ];\n');
            fprintf('obj.imu_gyro = [ raw_gyr(:,1),  raw_gyr(:,2), -raw_gyr(:,3) ];\n');
        else % Y-Up
            fprintf('obj.imu_acc  = [ raw_acc(:,1), -raw_acc(:,2),  raw_acc(:,3) ];\n');
            fprintf('obj.imu_gyro = [ raw_gyr(:,1), -raw_gyr(:,2),  raw_gyr(:,3) ];\n');
        end
    case 3
        if val < 0 % Z-Up (FLU) -> FRD
            fprintf('obj.imu_acc  = [ raw_acc(:,1), -raw_acc(:,2), -raw_acc(:,3) ];\n');
            fprintf('obj.imu_gyro = [ raw_gyr(:,1), -raw_gyr(:,2), -raw_gyr(:,3) ];\n');
        else % Z-Down
            fprintf('obj.imu_acc  = [ raw_acc(:,1),  raw_acc(:,2), -raw_acc(:,3) ];\n');
            fprintf('obj.imu_gyro = [ raw_gyr(:,1),  raw_gyr(:,2), -raw_gyr(:,3) ];\n');
        end
end
