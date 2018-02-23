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

function generateDecentrFrameStream(dataset_path, sequence_id, num_robots)
%% Load
root = [dataset_path '/' sequence_id '/'];
dpath = [root 'dslam/'];
load([dpath 'full_data_' num2str(num_robots) '_robots.mat']);

%% Individual streams
individual = cell(size(robot_vo_data));
for robot_i = 1:numel(individual)
    vo_data = robot_vo_data{robot_i};
    individual{robot_i} = cell(size(robot_vo_data{robot_i}.T_W_C));
    for frame_i = 1:numel(robot_vo_data{robot_i}.T_W_C)
        individual{robot_i}{frame_i}.robot_i = robot_i;
        if frame_i == 1
            individual{robot_i}{frame_i}.Sim_Cprev_C = ...
                eye(4);
        else
            individual{robot_i}{frame_i}.Sim_Cprev_C = ...
                vo_data.T_W_C{frame_i - 1} ^ -1 * vo_data.T_W_C{frame_i};
        end
        individual{robot_i}{frame_i}.original_T_O_C = ...
            vo_data.T_W_C{1} ^ -1 * vo_data.T_W_C{frame_i};
        individual{robot_i}{frame_i}.time = ...
            vo_data.times(frame_i) - vo_data.times(1);
        individual{robot_i}{frame_i}.netvlad = vo_data.netvlad(:, frame_i);
        individual{robot_i}{frame_i}.descs = vo_data.descs{frame_i};
        individual{robot_i}{frame_i}.wids3 = vo_data.wids3{frame_i};
        individual{robot_i}{frame_i}.wids4 = vo_data.wids4{frame_i};
        individual{robot_i}{frame_i}.lms_in_frame = ...
            vo_data.lms_in_frame{frame_i};
        
        individual{robot_i}{frame_i}.gt_T_W_C = ...
            vo_data.gt_T_W_C{frame_i};
    end
end

%% Spliced stream
spliced = vertcat(individual{:});
[~, sorting] = sort(cellfun(@(x) x.time, spliced));
decentr_stream = spliced(sorting);

%% Save
save([dpath 'decentr_stream_' num2str(num_robots) '_robots.mat'], ...
    'decentr_stream', '-v7.3');
end

