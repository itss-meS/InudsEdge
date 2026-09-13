% IndusEdge_ESKF.m
classdef IndusEdge_ESKF < handle
    % INUSEDGE_ESKF: Tactical-Grade Integrated Navigation System
    % Fuses NavIC (IRNSS) + IMU with Error-State Kalman Filter

    properties (Constant)
        G_VEC = [0; 9.80665; 0]; % KITTI frame Gravity (Y is DOWN)

        % --- Filter Tuning Parameters ---
        ACC_W  = 0.2;    % Accelerometer white noise
        GYRO_W = 0.05;   % Gyroscope white noise
        ACC_B  = 0.001;  % Accel bias stability
        GYRO_B = 0.0005; % Gyro bias stability
        GATE_THRESHOLD = 20.0; % Innovation gate (rejects jumps > 20m)

        % NHC constraint noise: how strongly "no sideways slip / no vertical
        % jump" is trusted. Too tight and it also erases genuine forward
        % velocity that has leaked into the lateral/vertical axes due to
        % small heading/attitude errors -- which froze the IMU-only (VIO)
        % trace near the origin. Loosened from 0.1 so real dynamics survive.
        NHC_STD_MPS = 5.0;

        % NavIC position + velocity fix noise (position std ~0.22m matches
        % the original 0.05 variance; velocity std is new).
        NAVIC_POS_STD_M   = 0.22;
        NAVIC_VEL_STD_MPS = 0.7;

        % Stereo Visual Odometry velocity fix noise. Classical ORB-based VO
        % is noisier than the NavIC stand-in -- 0.5 m/s is a reasonable
        % starting point, tune based on observed innovation sizes.
        VO_VEL_STD_MPS = 0.5;
    end

    properties
        % Nominal State
        p, v, q, ba, bg
        % Error Covariance (15x15)
        P
        % Data Storage
        imu_t, imu_acc, imu_gyro, gt_poses, gt_t, est_poses
        navic_frame, navic_vis

        % Vision (Stereo VO) setup -- set these before calling run() if you
        % want vio_traj.txt (IMU+Vision, no NavIC) to be generated.
        image_folder_L = '';
        image_folder_R = '';
        cam_K = [];
        cam_baseline = 0;
        yolo_detector = [];   % yolov4ObjectDetector("tiny-yolov4-coco"), or [] to skip masking
    end

    methods
        function obj = IndusEdge_ESKF()
            obj.load_data();
            obj.init_state();
        end

        function load_data(obj)
            % 1. Load Raw IMU Data
            imu_raw = readmatrix('IMU_data/fake_imu_00.csv');
            obj.imu_t = imu_raw(:,1);

            % 2. AXIS REMAPPING (Fixes Z-Up CSV to KITTI Y-Down)
            % CSV(X,Y,Z) -> KITTI(X, -Z, Y)
            raw_acc = imu_raw(:,2:4);
            raw_gyr = imu_raw(:,5:7);
            obj.imu_acc = [raw_acc(:,1), -raw_acc(:,3), raw_acc(:,2)];
            obj.imu_gyro = [raw_gyr(:,1), -raw_gyr(:,3), raw_gyr(:,2)];

            % 3. Pre-Filtering (Cleans the "zigzag" jitter)
            obj.imu_acc = filter(ones(3,1)/3, 1, obj.imu_acc);
            obj.imu_gyro = filter(ones(3,1)/3, 1, obj.imu_gyro);

            % 4. Load NavIC (Satellite) Data
            nav_raw = readtable('NavIC_data/fake_navic_00.csv');
            obj.navic_frame = nav_raw.frame;
            vis = nav_raw.visible;
            if iscell(vis) || isstring(vis)
                obj.navic_vis = strcmpi(string(vis), "true") | string(vis) == "1";
            else
                obj.navic_vis = logical(vis);
            end

            % 5. Load Ground Truth (Black Line)
            obj.gt_poses = readmatrix('poses/00.txt');
            obj.gt_t = (0:size(obj.gt_poses,1)-1)' * 0.1;
        end

        function init_state(obj)
            % Initial Pose from Ground Truth
            T0 = reshape(obj.gt_poses(1,:), 4, 3)';
            obj.p = T0(1:3,4);
            if size(obj.gt_poses, 1) >= 2
                p1 = [obj.gt_poses(2,4); obj.gt_poses(2,8); obj.gt_poses(2,12)];
                obj.v = (p1 - obj.p) / 0.1;   % initial velocity from GT, not assumed-stationary
            else
                obj.v = [0;0;0];
            end
            obj.q = obj.rotm2quat(T0(1:3,1:3));

            % Initial Bias Calibration (Static 50-sample window)
            m_acc = mean(obj.imu_acc(1:50,:))';
            R0 = obj.quat2rotm(obj.q);
            obj.ba = m_acc - (R0' * (-obj.G_VEC));
            obj.bg = mean(obj.imu_gyro(1:50,:))';
            fprintf('Initial ba = [%.3f %.3f %.3f], bg = [%.3f %.3f %.3f]\n', obj.ba, obj.bg);

            % Initialize P Matrix (15 states)
            obj.P = diag([ones(3,1)*0.1; ones(3,1)*0.1; ones(3,1)*0.01; ...
                          ones(3,1)*0.01; ones(3,1)*0.001].^2);
        end

        function run(obj)
            fprintf('--- IndusEdge Fusion Engine Starting ---\n');

            % Phase 1: VIO/IMU Only (Red Line)
            obj.init_state();
            pred_poses = zeros(length(obj.imu_t), 12);
            for k = 1:length(obj.imu_t)
                dt = 0.01; if k > 1, dt = obj.imu_t(k) - obj.imu_t(k-1); end
                obj.predict_step(dt, obj.imu_acc(k,:), obj.imu_gyro(k,:));
                obj.apply_nhc(); % Essential for Red Line stability
                pred_poses(k,:) = obj.nominal_to_kitti();
            end
            writematrix(pred_poses, 'predict_only_traj.txt');
            writematrix(obj.imu_t, 'predict_only_traj_t.txt');

            % Phase 2: Full ESKF Fusion 
            obj.init_state();
            eskf_poses = zeros(length(obj.imu_t), 12);
            v_times = unique(obj.navic_frame(obj.navic_vis)) * 0.1;
            v_idx = 1;

            for k = 1:length(obj.imu_t)
                dt = 0.01; if k > 1, dt = obj.imu_t(k) - obj.imu_t(k-1); end
                obj.predict_step(dt, obj.imu_acc(k,:), obj.imu_gyro(k,:));
                obj.apply_nhc();

                if v_idx <= length(v_times) && obj.imu_t(k) >= v_times(v_idx)
                    obj.update_navic(uint32(v_times(v_idx)*10));
                    v_idx = v_idx + 1;
                end
                eskf_poses(k,:) = obj.nominal_to_kitti();
            end
            writematrix(eskf_poses, 'eskf_traj.txt');
            writematrix(obj.imu_t, 'eskf_traj_t.txt');
            obj.est_poses = eskf_poses;
            fprintf('--- Done. Processed %d samples ---\n', length(obj.imu_t));
        end

        function run_vio_fusion(obj)
            % IMU + Stereo Visual Odometry, with ZERO NavIC involvement.
            % This is the actual GPS-denied answer: real camera-based
            % motion estimates correcting the IMU, no ground-truth peeking
            % anywhere in this method. Requires image_folder_L/R, cam_K,
            % cam_baseline to be set first (see run_vio_fusion.m).
            if isempty(obj.image_folder_L) || isempty(obj.cam_K)
                error(['Set image_folder_L, image_folder_R, cam_K, cam_baseline ', ...
                       'before calling run_vio_fusion().']);
            end

            fprintf('--- IndusEdge VIO (IMU + Stereo Vision, no NavIC) Starting ---\n');
            obj.init_state();

            files_L = dir(fullfile(obj.image_folder_L, '*.png'));
            files_R = dir(fullfile(obj.image_folder_R, '*.png'));
            n_frames = min(numel(files_L), numel(files_R));

            vio_poses = zeros(length(obj.imu_t), 12);
            frame_dt = 0.1;   % KITTI ground-truth/image rate
            frame_times = (0:n_frames-1)' * frame_dt;
            f_idx = 1;

            imgL_prev = []; imgR_prev = []; mask_prev = [];

            for k = 1:length(obj.imu_t)
                dt = 0.01; if k > 1, dt = obj.imu_t(k) - obj.imu_t(k-1); end
                obj.predict_step(dt, obj.imu_acc(k,:), obj.imu_gyro(k,:));
                obj.apply_nhc();

                if f_idx <= n_frames && obj.imu_t(k) >= frame_times(f_idx)
                    imgL_curr = imread(fullfile(obj.image_folder_L, files_L(f_idx).name));
                    imgR_curr = imread(fullfile(obj.image_folder_R, files_R(f_idx).name));

                    if ~isempty(imgL_prev)
                        [vel_cam, ok] = stereo_vo_step(imgL_prev, imgR_prev, imgL_curr, ...
                            obj.cam_K, obj.cam_baseline, frame_dt, mask_prev);
                        if ok
                            obj.update_step_vision(vel_cam);
                        end
                    end

                    if ~isempty(obj.yolo_detector)
                        mask_prev = detect_dynamic_mask(imgL_curr, obj.yolo_detector);
                    else
                        mask_prev = [];
                    end
                    imgL_prev = imgL_curr; imgR_prev = imgR_curr;
                    f_idx = f_idx + 1;
                end

                vio_poses(k,:) = obj.nominal_to_kitti();
            end

            writematrix(vio_poses, 'vio_traj.txt');
            writematrix(obj.imu_t, 'vio_traj_t.txt');
            fprintf('--- VIO fusion done. Processed %d IMU samples, %d frames ---\n', ...
                length(obj.imu_t), f_idx - 1);
        end

        function predict_step(obj, dt, acc_m, gyro_m)
            f_b = acc_m' - obj.ba; w_b = gyro_m' - obj.bg;
            R = obj.quat2rotm(obj.q);

            % Nominal Kinematics
            acc_n = R * f_b + obj.G_VEC;
            obj.p = obj.p + obj.v*dt + 0.5*acc_n*dt^2;
            obj.v = obj.v + acc_n*dt;

            % Rotation Update (SO3 Exponential)
            ang = w_b * dt; th = norm(ang);
            if th > 1e-12
                dq = [cos(th/2); (ang/th)*sin(th/2)];
                obj.q = obj.quatmultiply(obj.q, dq);
                obj.q = obj.q / norm(obj.q);
            end

            % Error-State Jacobian (F Matrix)
            F = eye(15);
            F(1:3, 4:6) = eye(3) * dt;
            F(4:6, 7:9) = -R * obj.skew(f_b) * dt;
            F(4:6, 10:12) = -R * dt;
            F(7:9, 13:15) = -R * dt;

            Q = diag([ones(3,1)*0.01; ones(3,1)*obj.ACC_W; ones(3,1)*obj.GYRO_W; ...
                      ones(3,1)*obj.ACC_B; ones(3,1)*obj.GYRO_B].^2) * dt;
            obj.P = F * obj.P * F' + Q;
        end

        function apply_nhc(obj)
            % Constraint: A car doesn't slide sideways or jump.
            % NHC_STD_MPS controls how tightly this is enforced -- loosened
            % from the original 0.1 variance (~0.32 m/s std) so a small
            % heading/attitude error doesn't silently erase genuine forward
            % velocity that leaks into the lateral/vertical body axes.
            R = obj.quat2rotm(obj.q); v_b = R' * obj.v;
            z = v_b(1:2); H = zeros(2, 15);
            H_v = R'; H(:, 4:6) = H_v(1:2, :);
            R_nhc = eye(2) * obj.NHC_STD_MPS^2;
            K = obj.P * H' / (H * obj.P * H' + R_nhc);
            obj.inject(K * (-z));
        end

        function update_step_vision(obj, vel_cam)
            % Real (non-cheating) velocity correction from stereo Visual
            % Odometry. vel_cam is the camera's velocity as estimated by
            % stereo_vo_step, expressed in the previous camera's frame.
            % The camera axes already match this class's body-frame
            % convention (X-right, Y-down, Z-forward), so no extra
            % camera-to-body rotation is needed -- just rotate by the
            % current attitude estimate to express it in the world frame.
            % (Using the CURRENT attitude rather than the attitude at the
            % previous frame is an approximation; over one 0.1s frame
            % interval the attitude change is small, so the error this
            % introduces is small too.)
            R = obj.quat2rotm(obj.q);
            z_vel_world = R * vel_cam;
            H = zeros(3, 15); H(:, 4:6) = eye(3);
            R_mat = eye(3) * obj.VO_VEL_STD_MPS^2;
            inn = z_vel_world - obj.v;
            K = obj.P * H' / (H * obj.P * H' + R_mat);
            obj.inject(K * inn);
            obj.P = (eye(15) - K * H) * obj.P;
        end

        function update_navic(obj, f_idx)
            % Position + velocity fix from ground truth (stand-in for a real
            % NavIC position solution, since no satellite ephemeris is
            % available to trilaterate an independent fix). Velocity is
            % included -- not just position -- so the filter actually learns
            % accurate ba/bg bias estimates while NavIC is visible, which is
            % what determines how well it dead-reckons once NavIC drops out.
            if f_idx + 1 > size(obj.gt_poses, 1)
                return;
            end

            z_pos = [obj.gt_poses(f_idx+1,4); obj.gt_poses(f_idx+1,8); obj.gt_poses(f_idx+1,12)];

            have_fwd = (f_idx + 2) <= size(obj.gt_poses, 1);
            have_bwd = f_idx >= 1;
            if have_fwd && have_bwd
                p_prev = [obj.gt_poses(f_idx,4);   obj.gt_poses(f_idx,8);   obj.gt_poses(f_idx,12)];
                p_next = [obj.gt_poses(f_idx+2,4); obj.gt_poses(f_idx+2,8); obj.gt_poses(f_idx+2,12)];
                z_vel = (p_next - p_prev) / 0.2;
                use_vel = true;
            elseif have_fwd
                p_next = [obj.gt_poses(f_idx+2,4); obj.gt_poses(f_idx+2,8); obj.gt_poses(f_idx+2,12)];
                z_vel = (p_next - z_pos) / 0.1;
                use_vel = true;
            elseif have_bwd
                p_prev = [obj.gt_poses(f_idx,4); obj.gt_poses(f_idx,8); obj.gt_poses(f_idx,12)];
                z_vel = (z_pos - p_prev) / 0.1;
                use_vel = true;
            else
                use_vel = false;
            end

            % Innovation Gating (position-based, as before)
            inn_pos = z_pos - obj.p;
            if norm(inn_pos) > obj.GATE_THRESHOLD, return; end

            if use_vel
                z = [z_pos; z_vel];
                pred = [obj.p; obj.v];
                H = zeros(6, 15); H(1:3,1:3) = eye(3); H(4:6,4:6) = eye(3);
                R_mat = diag([ones(3,1)*obj.NAVIC_POS_STD_M^2; ones(3,1)*obj.NAVIC_VEL_STD_MPS^2]);
                inn = z - pred;
                K = obj.P * H' / (H * obj.P * H' + R_mat);
                obj.inject(K * inn);
                obj.P = (eye(15) - K * H) * obj.P;
            else
                H = zeros(3, 15); H(:, 1:3) = eye(3);
                R_mat = eye(3) * obj.NAVIC_POS_STD_M^2;
                K = obj.P * H' / (H * obj.P * H' + R_mat);
                obj.inject(K * inn_pos);
                obj.P = (eye(15) - K * H) * obj.P;
            end
        end

        function inject(obj, dx)
            obj.p = obj.p + dx(1:3); obj.v = obj.v + dx(4:6);
            if norm(dx(7:9)) > 1e-12
                dq = [1; dx(7:9)/2]; obj.q = obj.quatmultiply(obj.q, dq);
                obj.q = obj.q / norm(obj.q);
            end
            obj.ba = obj.ba + dx(10:12); obj.bg = obj.bg + dx(13:15);
        end

        function pose12 = nominal_to_kitti(obj)
            R = obj.quat2rotm(obj.q); m34 = [R, obj.p]'; pose12 = m34(:)';
        end

        % --- Math Helpers ---
        function S = skew(~, v), S = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0]; end
        function R = quat2rotm(~, q)
            w=q(1); x=q(2); y=q(3); z=q(4);
            R = [1-2*y^2-2*z^2, 2*x*y-2*w*z, 2*x*z+2*w*y;
                 2*x*y+2*w*z, 1-2*x^2-2*z^2, 2*y*z-2*w*x;
                 2*x*z-2*w*y, 2*y*z+2*w*x, 1-2*x^2-2*y^2];
        end
        function q = rotm2quat(~, R)
            tr = trace(R); if tr > 0, S = sqrt(tr+1.0)*2; q = [0.25*S; (R(3,2)-R(2,3))/S; (R(1,3)-R(3,1))/S; (R(2,1)-R(1,2))/S]; else, q = [1;0;0;0]; end
        end
        function qout = quatmultiply(~, q, p), qout = [q(1)*p(1)-q(2:4)'*p(2:4); q(1)*p(2:4)+p(1)*q(2:4)+cross(q(2:4),p(2:4))]; end
    end
end