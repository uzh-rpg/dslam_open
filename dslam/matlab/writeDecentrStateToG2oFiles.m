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

function writeDecentrStateToG2oFiles(decentr_state, outputDir, ...
    group_reindexing)
% Inspired by Luca Carlone's
% https://bitbucket.org/lucacarlone/pgo3d-duality-opencode/src/ebb6e1b8cebaad7f2aaf581b1d0c0bad737faebb/lib/writeG2oDataset3D.m?at=master&fileviewer=file-view-default

nr_robots = numel(decentr_state);

for robot_i = 1:nr_robots
    file_ids(robot_i) = fopen(...
        [outputDir '/' num2str(robot_i - 1) '.g2o'], 'w');
end

for robot_i = 1:nr_robots
    robot_state = decentr_state{robot_i};
    nr_poses = numel(robot_state.Sim_O_C);
    file_id = file_ids(robot_i);
    
    % add poses from Sim_O_C to initial
    for pose_i = 1:nr_poses
        T_O_C = robot_state.Sim_O_C{pose_i};
        writeG2oPose(file_id, robot_i, pose_i, T_O_C);
    end
    
    % generate odometry graph from pose_i - 1 to pose_i
    for pose_i = 1:nr_poses-1
        % Really, this field is a misnomer - should be Sim_C_Cnext - sorry!
        relative_pose = robot_state.Sim_Cprev_C{pose_i};
        writeG2oConstraint(file_id, gtsamFrameId(robot_i, pose_i), ...
            gtsamFrameId(robot_i, pose_i + 1), relative_pose, eye(6));
    end
    
    % add inter-robot edges to graphs
    for match_i = 1:numel(robot_state.place_matches)
        place_match = robot_state.place_matches{match_i};
        
        if isempty(place_match) || group_reindexing(place_match.robot_i) < robot_i
            continue;
        end
        
        matched_robot = group_reindexing(place_match.robot_i);
        assert(any(...
            decentr_state{robot_i}.grouped_with == place_match.robot_i)...
            || robot_i == group_reindexing(place_match.robot_i));
        if (matched_robot <= 0 || matched_robot > nr_robots)
            nr_robots
            place_match.robot_i
            group_reindexing
            matched_robot
            assert(false);
        end
        
        % add edge to both the graphs
        query_id = gtsamFrameId(robot_i, match_i);
        match_id = gtsamFrameId(matched_robot, place_match.frame_i);
        T_Q_M = tInv(place_match.Sim_M_Q);
        
        writeG2oConstraint(file_id, query_id, match_id, T_Q_M, eye(6));
        % The following doesn't work!:
        %writeG2oConstraint(file_ids(place_match.robot_i), match_id, query_id, T_Q_M^-1, eye(6));
        writeG2oConstraint(file_ids(matched_robot), query_id, match_id, T_Q_M, eye(6));
    end
end

end
