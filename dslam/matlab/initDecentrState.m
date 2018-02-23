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

function decentr_state = initDecentrState(full_state)

num_robots = numel(full_state);

decentr_state = cell(size(full_state));
for i = 1:num_robots
    decentr_state{i}.Sim_W_O = eye(4);
    decentr_state{i}.Sim_O_C = cell(0);
    decentr_state{i}.Sim_Cprev_C = cell(0);
    decentr_state{i}.original_T_O_C = cell(0);
    decentr_state{i}.times = [];
    decentr_state{i}.netvlad = [];
    decentr_state{i}.descs = {};
    decentr_state{i}.wids3 = {};
    decentr_state{i}.wids4 = {};
    decentr_state{i}.lms_in_frame = cell(0);
    decentr_state{i}.gt_T_W_C = cell(0);
    decentr_state{i}.T_gt_O_ate = full_state{i}.T_W_C{1};
    
    % DVPR related members
    decentr_state{i}.dvpr_queries_netvlad = [];
    decentr_state{i}.dvpr_queries_robot_i = [];
    decentr_state{i}.dvpr_queries_frame_i = [];
    
    % Robust relpose
    decentr_state{i}.consistent_groups = cell(num_robots, 1);
    for j = 1:num_robots
        decentr_state{i}.consistent_groups{j}.members = {};
        decentr_state{i}.consistent_groups{j}.floating = true;
    end
    
    decentr_state{i}.place_matches = cell(0);
    decentr_state{i}.grouped_with = [];
    decentr_state{i}.converged = true;
    % Pre-loading landmarks -> more managable
    T_O_W = full_state{i}.T_W_C{1} ^ -1;
    decentr_state{i}.p_O_lm = ...
        T_O_W(1:3, 1:3) * full_state{i}.p_W_lm' + T_O_W(1:3, 4);
end

end

