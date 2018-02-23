% Copyright (C) 2017-2018 Titus Cieslewski, RPG, University of Zurich, 
%   Switzerland
%   You can contact the author at <titus at ifi dot uzh dot ch>
% Copyright (C) 2017-2018 Siddharth Choudhary, College of Computing,
%   Georgia Institute of Technology, Atlanta, GA, USA
% Copyright (C) 2017-2018 Davide Scaramuzza, RPG, University of Zurich, 
%   Switzerland
%
% This file is part of dslam_open.
%
% dslam_open is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% dslam_open is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with dslam_open. If not, see <http://www.gnu.org/licenses/>.

function [vo_data] = getVOData(dataset_path, sequence_id, data_type)

root = [dataset_path '/' sequence_id '/'];
dpath = [root 'dslam/'];

pose_file = [dpath 'poses_0.txt'];
time_file = [dpath 'times_0.txt'];
lm_pos_file = [dpath 'lm_pos_0.txt'];
lm_obs_file = [dpath 'lm_obs_0.txt'];
desc_file = [dpath 'descs_0.txt'];

if (strcmp(data_type, 'kitti'))
    gt_pose_file = [dataset_path '/poses/' sequence_id '.txt'];
    gt_times_file = [root 'times.txt'];
else
    assert(strcmp(data_type, 'stata'));
    gt_pose_file = [root 'gt_poses.txt'];
    gt_times_file = [root 'times.txt'];
end

assert(exist(pose_file, 'file') == 2);
assert(exist(time_file, 'file') == 2);
assert(exist(lm_pos_file, 'file') == 2);
assert(exist(lm_obs_file, 'file') == 2);
assert(exist(desc_file, 'file') == 2);
assert(exist(gt_pose_file, 'file') == 2);
assert(exist(gt_times_file, 'file') == 2);

%%
T_W_C_raw = load(pose_file);
vo_data.n_frames = size(T_W_C_raw, 1);
vo_data.T_W_C = cell(vo_data.n_frames, 1);

for i = 1:vo_data.n_frames
    vo_data.T_W_C{i} = reshape(T_W_C_raw(i, :), 4, 4)';
end

%%
vo_data.times = load(time_file);
vo_data.p_W_lm = load(lm_pos_file);
vo_data.n_lm = size(vo_data.p_W_lm, 1);

vo_data.lms_in_frame = cell(vo_data.n_frames, 1);
vo_data.descs = cell(vo_data.n_frames, 1);
lm_obs = load(lm_obs_file);
descs_and_wids = load(desc_file);
descs = uint8(descs_and_wids(:, 1:end-2));
wids3 = uint64(descs_and_wids(:, end-1));
wids4 = uint64(descs_and_wids(:, end));
for i = 1:vo_data.n_frames
    vo_data.lms_in_frame{i} = lm_obs(lm_obs(:, 1) == (i - 1), 2) + 1;
    vo_data.descs{i} = descs(lm_obs(:, 1) == (i - 1), :);
    vo_data.wids3{i} = wids3(lm_obs(:, 1) == (i - 1), :);
    vo_data.wids4{i} = wids4(lm_obs(:, 1) == (i - 1), :);
end

%% Get ground truth poses
if (strcmp(data_type, 'kitti'))
    gt_times = load(gt_times_file);
    gt_T_W_C_raw = load(gt_pose_file);
else
    assert(strcmp(data_type, 'stata'));
    gt_times = load(gt_times_file) / 1e9;
    gt_T_W_C_raw = load(gt_pose_file);
end
%%

[~, gt_indices] = pdist2(...
    gt_times, vo_data.times, 'squaredeuclidean', 'Smallest', 1);
gt_T_W_C_raw = gt_T_W_C_raw(gt_indices, :);
vo_data.gt_T_W_C = cell(vo_data.n_frames, 1);
for i = 1:vo_data.n_frames
    vo_data.gt_T_W_C{i} = [reshape(gt_T_W_C_raw(i, :), 4, 3)'; 0 0 0 1];
end
end
