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

function [match_accepted, consistent_group] = ...
    acceptMatchIfConsistent(match, query_frame_i, query_robot_i, ...
    decentr_state, params)

this.frame_i = query_frame_i;
this.match = match;

robot_state = decentr_state{query_robot_i};
consistent_group = robot_state.consistent_groups{match.robot_i};

% Remove out-of-range matches from consistent group.
in_range = cellfun(@(x) areFramesWithinRange(robot_state, ...
            x.frame_i, query_frame_i, ...
            params.robust_relpose_consistency_range), ...
            consistent_group.members);
consistent_group.members = consistent_group.members(in_range);

% If all members were removed, group is forcibly floating again.
if (all(~in_range))
    consistent_group.floating = true;
end

% If consistent (or group empty), add current match.
consistent = cellfun(@(x) areMatchesConsistent(decentr_state, ...
    query_robot_i, x, this, params.robust_relpose_position_tolerance), ...
    consistent_group.members);
if (all(consistent))  % all([]) = true according to spec
    consistent_group.members = [consistent_group.members; this];
elseif (consistent_group.floating)
    % If not consistent and group was floating, replace group with this.
    consistent_group.members = {this};
    match_accepted = false;
    return;
else
    match_accepted = false;
    return;
end

% We only get here if the match has been added to the consitent group.

if ~consistent_group.floating
    match_accepted = true;
else
    if numel(consistent_group.members) > params.robust_relpose_min_group_size
        consistent_group.floating = false;
        match_accepted = true;
    else
        match_accepted = false;
    end
end

end
