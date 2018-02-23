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

function [accuracy, decentr_state] = evalAccuracy(
    decentr_state, opt_handle, time)

% Structure:
% - time
% - matched_groups:
%   - members
%   - num_frames
% - optimized_groups:
%   - members
%   - num_frames
%   - ATE

accuracy.time = time;

n_robots = numel(decentr_state);

matched_group_assignment = 1:n_robots;
for i = 1:n_robots
    matched_group_assignment(i) = ...
        min([decentr_state{i}.grouped_with i]);
end
groups = unique(matched_group_assignment);
for i = 1:numel(groups)
    group_index = groups(i);
    accuracy.matched_groups(i).members = ...
        find(matched_group_assignment == group_index);
    accuracy.matched_groups(i).num_frames = 0;
    for member_i = accuracy.matched_groups(i).members
        accuracy.matched_groups(i).num_frames = ...
            accuracy.matched_groups(i).num_frames + ...
            numel(decentr_state{member_i}.Sim_O_C);
    end
end

accuracy.optimized_groups = opt_handle.optimized_groups;
for group_i = 1:numel(accuracy.optimized_groups)
    members = accuracy.optimized_groups(group_i).members;
    [accuracy.optimized_groups(group_i).num_frames, ...
        accuracy.optimized_groups(group_i).ATE, decentr_state] = ...
        getConnectedAte(decentr_state, members);
end

end

