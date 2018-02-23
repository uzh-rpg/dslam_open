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

function splitAndSave(dataset_path, sequence_id, num_robots, overlap)
%%
root = [dataset_path '/' sequence_id '/'];
dpath = [root 'dslam/'];

load([dpath 'full_data.mat']);
robot_vo_data = cell(num_robots, 1);

%% Frame assignment:
if mod(vo_data.n_frames, num_robots) ~= 0
    n_frames_padded = ...
        vo_data.n_frames + num_robots - mod(vo_data.n_frames, num_robots);
else
    n_frames_padded = vo_data.n_frames
end
assignments1 = reshape(1:n_frames_padded, [], num_robots);
assignments1 = mat2cell(assignments1', ones(1, num_robots));
assignments1{end}(assignments1{end} > vo_data.n_frames) = [];

for i = 1:num_robots
    assignments{i} = assignments1{i};
    if i > 1
        assignments{i} = ...
            [assignments1{i-1}(end-overlap+1:end) assignments{i}];
    end
    if i < num_robots
        assignments{i} = ...
            [assignments{i} assignments1{i+1}(1:overlap)];
    end
end

%% Split up:
for i = 1:num_robots
    robot_vo_data{i}.T_W_C = vo_data.T_W_C(assignments{i});
    robot_vo_data{i}.times = vo_data.times(assignments{i});
    robot_vo_data{i}.netvlad = vo_data.netvlad(:, assignments{i});
    robot_vo_data{i}.descs = vo_data.descs(assignments{i});
    robot_vo_data{i}.wids3 = vo_data.wids3(assignments{i});
    robot_vo_data{i}.wids4 = vo_data.wids4(assignments{i});
    robot_vo_data{i}.gt_T_W_C = vo_data.gt_T_W_C(assignments{i});
    robot_vo_data{i}.lms_in_frame = vo_data.lms_in_frame(assignments{i});
    robot_vo_data{i}.p_W_lm = vo_data.p_W_lm;
end

%% Cull landmarks:
for robot_i = 1:num_robots
    observed_lms = [];
    n_frames = numel(robot_vo_data{robot_i}.lms_in_frame);
    for frame_i = 1:n_frames
        observed_lms = unique([...
            observed_lms; robot_vo_data{robot_i}.lms_in_frame{frame_i}]);
    end
    observed_lms = sort(observed_lms);
    lm_reindexing = zeros(numel(robot_vo_data{robot_i}.p_W_lm), 1);
    lm_reindexing(observed_lms) = 1:numel(observed_lms);
    robot_vo_data{robot_i}.p_W_lm = ...
        robot_vo_data{robot_i}.p_W_lm(observed_lms, :);
    for frame_i = 1:n_frames
        robot_vo_data{robot_i}.lms_in_frame{frame_i} = ...
            lm_reindexing(robot_vo_data{robot_i}.lms_in_frame{frame_i});
    end
end

%%
save([dpath 'full_data_' num2str(num_robots) '_robots.mat'], ...
    'robot_vo_data', '-v7.3');
end

