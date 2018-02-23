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

function generateVerificationRequests(...
    dataset_path, sequence_id, num_robots)

%%
root = [dataset_path '/' sequence_id '/'];
dpath = [root 'dslam/'];
robot_vo_data = [];
load([dpath 'full_data_' num2str(num_robots) '_robots.mat']);

%%
p_W_lm = robot_vo_data{1}.p_W_lm;
tot_frames = sum(cellfun(@(x) numel(x.T_W_C), robot_vo_data));
verification_request = cell(tot_frames, 1);
vr_i = 1;

for robot_i = 1:num_robots
    robot_vo_data{robot_i}.id = robot_i;
end

dists = cell(num_robots, 1);
%%
for robot_i = 1:num_robots
    %%
    other_vlads = cellfun(@(x) x.netvlad, ...
        robot_vo_data(1:num_robots ~= robot_i), 'UniformOutput', false);
    other_vlads = cell2mat(other_vlads');
    [dists{robot_i}, matches] = pdist2(other_vlads(1:128, :)', ...
        robot_vo_data{robot_i}.netvlad(1:128, :)', ...
        'squaredeuclidean', 'Smallest', 1);
    
    other_descs = cellfun(@(x) x.descs, ...
        robot_vo_data(1:num_robots ~= robot_i), 'UniformOutput', false);
    other_descs = vertcat(other_descs{:});
    
    other_lms = cellfun(@(x) x.lms_in_frame, ...
        robot_vo_data(1:num_robots ~= robot_i), 'UniformOutput', false);
    other_lms = vertcat(other_lms{:});
    
    other_T_W_C = cellfun(@(x) x.T_W_C, ...
        robot_vo_data(1:num_robots ~= robot_i), 'UniformOutput', false);
    other_T_W_C = vertcat(other_T_W_C{:});
    
    other_robot_i = cellfun(@(x) x.id * ones(numel(x.T_W_C), 1), ...
        robot_vo_data(1:num_robots ~= robot_i), 'UniformOutput', false);
    other_robot_i = cell2mat(other_robot_i);
    
    other_frame_i = cellfun(@(x) (1:numel(x.T_W_C))', ...
        robot_vo_data(1:num_robots ~= robot_i), 'UniformOutput', false);
    other_frame_i = cell2mat(other_frame_i);
    
    matched_descs = other_descs(matches);
    matched_lms = other_lms(matches);
    matched_T_W_C = other_T_W_C(matches);
    matched_robot_i = other_robot_i(matches);
    matched_frame_i = other_frame_i(matches);
    
    %%
    for frame_i = find(dists{robot_i} < 0.01)
        %%
        % Cat self
        T_W_C = robot_vo_data{robot_i}.T_W_C{frame_i};
        T_C_W = T_W_C^-1;
        p_C_lm_q = (T_C_W(1:3, 1:3) * p_W_lm(...
            robot_vo_data{robot_i}.lms_in_frame{frame_i}, :)' + ...
            T_C_W(1:3, 4))';
        descs_q = robot_vo_data{robot_i}.descs{frame_i};
        num_kp = size(descs_q, 1);
        verification_request{vr_i} = ...
            [repmat([robot_i-1, frame_i-1, 0], [num_kp, 1]) ...
            descs_q p_C_lm_q];
        vr_i = vr_i + 1;
        
        % Cat match
        orob_i = matched_robot_i(frame_i);
        ofra_i = matched_frame_i(frame_i);
        T_W_C = robot_vo_data{orob_i}.T_W_C{ofra_i};
        T_C_W = T_W_C^-1;
        p_C_lm_m = (T_C_W(1:3, 1:3) * p_W_lm(...
            robot_vo_data{orob_i}.lms_in_frame{ofra_i}, :)' + ...
            T_C_W(1:3, 4))';
        descs_m = robot_vo_data{orob_i}.descs{ofra_i};
        num_kp = size(descs_m, 1);
        verification_request{vr_i} = ...
            [repmat([orob_i-1, ofra_i-1, 1], [num_kp, 1]) descs_m p_C_lm_m];
        vr_i = vr_i + 1;
    end
end

% From below: 0.01 is a good threshold.
%dists = cell2mat(dists');
%figure(1);
%histogram(dists);

verification_request = cell2mat(verification_request);

%%
dlmwrite([dpath 'verification_request_' num2str(num_robots) '.txt'],...
    verification_request, ' ');
end

