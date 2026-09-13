% Usage:


% Output files (written to the current folder):
%   predict_only_traj.txt  -- IMU dead-reckoning only (Step 1 verification)
%   eskf_traj.txt          -- Full ESKF with NavIC updates
%   eskf_eval.mat          -- Evaluation data for plotting

eskf = IndusEdge_ESKF();
eskf.run();
