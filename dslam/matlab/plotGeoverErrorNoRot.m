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

function plotGeoverErrorNoRot(decentr_state, fig_num, label)

%% Build struct with rel poses and ground truth
rel_poses = {};
for robot_i = 1:numel(decentr_state)
    r_i_data = decentr_state{robot_i};
    for query_i = 1:numel(r_i_data.place_matches)
        match = r_i_data.place_matches{query_i};
        if (numel(match) > 0)
            rel_pose = [];
            rel_pose.Sim_M_Q = match.Sim_M_Q;
            rel_pose.est_dist = norm(match.Sim_M_Q(1:3, 4));
            gt_p_W_M = ...
                decentr_state{match.robot_i}.gt_T_W_C{...
                match.frame_i}(1:3, 4);
            gt_p_W_Q = r_i_data.gt_T_W_C{query_i}(1:3, 4);
            rel_pose.gt_dist = ...
                norm(gt_p_W_M - gt_p_W_Q);
            rel_poses = [rel_poses; {rel_pose}];
        end
    end
end

%% Eval errors: position
figure(fig_num);
pos_error = cellfun(@(x) norm(x.est_dist - x.gt_dist), rel_poses);
subplot(1, 2, 1);
plot(pos_error);
title([label ': position errors']);
xlabel('rel pose #');
ylabel('position error [m]');

subplot(1, 2, 2);
plot(sort(pos_error), 1:numel(pos_error));
title([label ': position errors'])
xlabel('position error [m]');
ylabel('num rel poses with lower error');
end
