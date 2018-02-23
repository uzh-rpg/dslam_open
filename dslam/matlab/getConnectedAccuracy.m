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

function connected_accuracy = getConnectedAccuracy(...
    decentr_state, robot_i)
% For each pose P in the connected component of robot_i, the position error 
% between estimated and ground truth p_C_P, where C is the latest pose of 
% robot_i.

%%
group = [robot_i, decentr_state{robot_i}.grouped_with];
%%
Sim_C_W = (decentr_state{robot_i}.Sim_W_O * ...
    decentr_state{robot_i}.Sim_O_C{end}) ^ -1;
Sim_W_P = {};
for i = group
    Sim_W_P = [Sim_W_P; ...
        cellfun(@(x) decentr_state{i}.Sim_W_O * x, ...
        decentr_state{i}.Sim_O_C, 'UniformOutput', false)];
end
Sim_C_P = cellfun(@(x) Sim_C_W * x, Sim_W_P, 'UniformOutput', false);
p_C_P_estim = cellfun(@(x) x(1:3, 4)', Sim_C_P, 'UniformOutput', false);
p_C_P_estim = cell2mat(p_C_P_estim);

%%
T_C_W = decentr_state{robot_i}.gt_T_W_C{end} ^ -1;
T_W_P = {};
for i = group
    T_W_P = [T_W_P; decentr_state{i}.gt_T_W_C];
end
T_C_P = cellfun(@(x) T_C_W * x, T_W_P, 'UniformOutput', false);
p_C_P_gt = cellfun(@(x) x(1:3, 4)', T_C_P, 'UniformOutput', false);
p_C_P_gt = cell2mat(p_C_P_gt);

%%
error = p_C_P_estim - p_C_P_gt;
error = sqrt(sum(error.^2, 2));

connected_accuracy = [robot_i, error'];

end
