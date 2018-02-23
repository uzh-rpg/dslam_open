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

function writeGroundTruthToG2oFiles(robot_vo_data, outputDir)

mkdir(outputDir);
nr_robots = numel(robot_vo_data);

for robot_i = 1:nr_robots
    file_ids(robot_i) = fopen(...
        [outputDir '/' num2str(robot_i - 1) '.g2o'], 'w');
end

for robot_i = 1:nr_robots
    robot_state = robot_vo_data{robot_i};
    nr_poses = numel(robot_state.gt_T_W_C);
    file_id = file_ids(robot_i);
    
    for pose_i = 1:nr_poses
        T_O_C = robot_state.gt_T_W_C{pose_i};
        writeG2oPose(file_id, robot_i, pose_i, T_O_C);
    end
end

end
