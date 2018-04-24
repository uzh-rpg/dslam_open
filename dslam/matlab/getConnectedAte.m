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

function [num_frames, connected_ate, decentr_state] = getConnectedAte(...
    decentr_state, group)
%% Poses to positions ground truth.
p_gt_C = [];
for i = group
    p_gt_C = [p_gt_C; cell2mat(cellfun(@(x) x(1:3, 4)', ...
        decentr_state{i}.gt_T_W_C, 'UniformOutput', false))];
end
%% Poses to positions group estimate.
p_gW_C = [];
for i = group
    T_gW_C = cellfun(@(x) decentr_state{i}.Sim_W_O * x, ...
        decentr_state{i}.Sim_O_C, 'UniformOutput', false);
    p_gW_C = [p_gW_C; cell2mat(cellfun(@(x) x(1:3, 4)', ...
        T_gW_C, 'UniformOutput', false))];
end

%% Optimize: min error between positions, func of T_gt_O_ate
[decentr_state{min(group)}.T_gt_O_ate, connected_ate] = ...
    alignTrajs(p_gt_C', p_gW_C', decentr_state{min(group)}.T_gt_O_ate);
num_frames = size(p_gW_C, 1);

end
