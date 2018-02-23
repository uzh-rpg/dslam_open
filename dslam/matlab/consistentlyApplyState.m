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

function decentr_state = consistentlyApplyState(...
        updated_state, decentr_state)
% Apply updated state that might have less frames per robot than the
% decentr_state.
n_robots = numel(decentr_state);
assert(numel(updated_state) == n_robots);

for robot_i = 1:n_robots
    if (isempty(updated_state{robot_i}))
        continue;
    end
    
    updated = updated_state{robot_i}.Sim_O_C;
    n_updated_frames = numel(updated);
    n_frames = numel(decentr_state{robot_i}.Sim_O_C);
    assert(n_updated_frames <= n_frames);
    
    for frame_i = 1:n_updated_frames-1
        decentr_state{robot_i}.Sim_O_C{frame_i} = updated{frame_i};
    end
    
    % Last one is a special since we look for a transform that we can apply
    % to all following transforms to keep relative poses consistent.
    % We seek T_update such that T_O_C_new = T_update * T_O_C_old
    T_update = fixT(updated{n_updated_frames} * ...
        tInv(decentr_state{robot_i}.Sim_O_C{n_updated_frames}));
    
    for frame_i = n_updated_frames:n_frames
        decentr_state{robot_i}.Sim_O_C{frame_i} = fixT(T_update * ...
            decentr_state{robot_i}.Sim_O_C{frame_i});
    end
end

end

