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

function decentr_state = readDecentrStateFromOptG2oFiles(...
    g2o_dir, decentr_state, suffix)
% Inspired by Luca Carlone's
% https://bitbucket.org/lucacarlone/pgo3d-duality-opencode/src/ebb6e1b8cebaad7f2aaf581b1d0c0bad737faebb/lib/readG2oDataset3D.m?at=master&fileviewer=file-view-default

nr_robots = numel(decentr_state);

for robot_i = 1:nr_robots
    file_id = fopen(...
        [g2o_dir '/' num2str(robot_i - 1) suffix '.g2o'], 'r');
    
    while true
        line = fgets(file_id);
        data = textscan(line, '%s %d64 %f %f %f %f %f %f %f');
        if ~strcmp(data{1}, 'VERTEX_SE3:QUAT')
            break
        end
        [robot_i_val, frame_i] = gtsamFrameIdToIndices(data{2});
        assert(robot_i_val == robot_i);
        
        x = data{3}; y = data{4}; z = data{5};
        qx = data{6}; qy = data{7}; qz = data{8}; qw = data{9};
        
        Sim_W_C = eye(4);
        Sim_W_C(1:3, 4) = [x y z]';
        q = [qw, qx qy, qz]';
        if(abs(norm(q)-1) > 1e-3)
            norm(q)
            error('Quaternion has not unit norm');
        else
            q = q/norm(q); % we normalize anyway
        end
        
        Sim_W_C(1:3, 1:3) = fixR(quat2rot(q));
        
        decentr_state{robot_i}.Sim_O_C{frame_i} = ...
            Sim_W_C;
    end
end

end

