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

function parseVerificationResponse(dataset_path, sequence_id, num_robots)

root = [dataset_path '/' sequence_id '/'];
dpath = [root 'dslam/'];

%% Load data
load([dpath 'full_data_' num2str(num_robots) '_robots.mat']);
% Format: query_robot query_frame match_robot match_frame success Sim_M_Q
verification_response = load(...
    [dpath 'verification_response_' num2str(num_robots) '.txt']);
% Filter responses to only successful ones
verification_response(verification_response(:, 5) == 0, :) = [];

%% Parse verification data
Sim_M_Q = cell(size(verification_response, 1), 1);
for i = 1:size(verification_response, 1)
    Sim_M_Q{i} = reshape(verification_response(i, 6:end), [4 4]);
end

%% Embed place recognition in robot data
robot_data = robot_vo_data;
for i = 1:numel(robot_data)
    robot_data{i}.place_matches = cell(size(robot_data{i}.descs));
end
for i = 1:size(verification_response, 1)
    q_robot_i = verification_response(i, 1) + 1;
    q_frame_i = verification_response(i, 2) + 1;
    m_robot_i = verification_response(i, 3) + 1;
    m_frame_i = verification_response(i, 4) + 1;
    
    robot_data{q_robot_i}.place_matches{q_frame_i}.robot_i = m_robot_i;
    robot_data{q_robot_i}.place_matches{q_frame_i}.frame_i = m_frame_i;
    robot_data{q_robot_i}.place_matches{q_frame_i}.Sim_M_Q = Sim_M_Q{i};
end

%% Visualize geometrically verified places
plotGlobalConfusion(robot_data);
title('Global confusion after GV');

%%
save([dpath 'robot_data_' num2str(num_robots) '.mat'], 'robot_data', ...
    '-v7.3');

end
