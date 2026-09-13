# IndusEdge_ESKF

MATLAB scaffold for the NavIC/INS tightly-coupled Error-State Kalman Filter.

## Folder structure

```
IndusEdge_ESKF/
├── IndusEdge_ESKF.m           # ESKF class (state, predict, update, eval)
├── run_eskf.m                 # Top-level runner script
├── align_traj_to_gt.m         # Helper: time-aligns IMU-rate trajectory to GT timestamps
├── IndusEdge_VIO_NavIC_GIF.m  # Animated GIF: Ground Truth + VIO + NavIC + EKF, next to camera feed
├── IndusEdge_GT_vs_EKF_GIF.m  # Animated GIF: Ground Truth vs EKF only, with live error/RMSE
├── NavIC_data/
│   └── fake_navic_00.csv      # frame, sat_id, pseudorange_m, visible  <- drop file here
├── IMU_data/
│   └── fake_imu_00.csv        # timestamp, accel_x/y/z, gyro_x/y/z (100 Hz)  <- drop file here
└── poses/
    └── 00.txt                 # KITTI ground truth, 12 numbers/line (3x4 R|t)  <- drop file here
```

The `NavIC_data/`, `IMU_data/`, and `poses/` folders are currently empty —
copy in the three CSV/TXT files listed above (matching the exact filenames)
before running.

## Running

```matlab
run_eskf
```

This runs Phase 1 (predict-only dead-reckoning) then Phase 2 (full ESKF with
NavIC updates), and writes:

- `predict_only_traj.txt` + `predict_only_traj_t.txt` — IMU-only dead-reckoning trajectory and its timestamps (expect visible drift)
- `eskf_traj.txt` + `eskf_traj_t.txt` — filtered trajectory and its timestamps (should track ground truth when satellites are visible)
- `eskf_eval.mat` — evaluation data (ATE RMSE + arrays for plotting)

These trajectories are saved at IMU rate (100 Hz); ground truth is at ~10 Hz.
The `_t.txt` companion files record the real timestamp of each row so the
GIF scripts below can align everything correctly via `align_traj_to_gt.m`.

## Animated comparisons

After `run_eskf` has produced its output files, set `image_folder` at the
top of either script to your KITTI image directory, then run:

```matlab
IndusEdge_VIO_NavIC_GIF     % Ground Truth + VIO + NavIC + EKF, next to camera feed
IndusEdge_GT_vs_EKF_GIF     % Ground Truth vs EKF only, with live error/RMSE readout
```

Each writes its own `.gif` file into the project folder.

## Notes / TODOs left in the scaffold

- `NaN` pseudoranges (where `visible = False`) are skipped automatically in the update loop.
- IMU is at 100 Hz, not the paper's 400 Hz — regenerate data if 400 Hz is required.
- `get_satellite_position()`, `nav_to_ecef()`, and `ecef_to_nav_rot()` are placeholders — wire up real ephemeris/geodesy before trusting NavIC update numbers.
- `update_step_vision()` is a stub for the vision/VIO update, pending feature tracks.
