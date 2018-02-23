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

function plotDecentrState(decentr_state, plot_gt)
%%
N = numel(decentr_state);
if (nargin < 2)
    plot_gt = true;
end

robot_xz = cell(size(decentr_state));
for i = 1:numel(robot_xz)
    Sim_W_C = cellfun(@(x) decentr_state{i}.Sim_W_O * x, ...
        decentr_state{i}.Sim_O_C, 'UniformOutput', false);
    robot_xz{i} = cellfun(@(x) ...
        [x(1, 4) x(3,4)], Sim_W_C, 'UniformOutput', false);
    robot_xz{i} = cell2mat(robot_xz{i});
end

robot_xz_gt = cell(size(decentr_state));
for i = 1:numel(robot_xz_gt)
    gt_T_W_C = cellfun(@(x) tInv(decentr_state{1}.T_gt_O_ate) * x, ...
        decentr_state{i}.gt_T_W_C, 'UniformOutput', false);
    robot_xz_gt{i} = cellfun(@(x) ...
        [x(1, 4) x(3,4)], gt_T_W_C, 'UniformOutput', false);
    robot_xz_gt{i} = cell2mat(robot_xz_gt{i});
end

%%
if (any(cellfun(@(x) numel(x) == 0, robot_xz)))
    return
end

clf;
hold on;
% Plot ground truth
c = colormap('lines');
if (plot_gt)
    for i = 1:numel(robot_xz_gt)
        plot(robot_xz_gt{i}(:, 1), robot_xz_gt{i}(:, 2), '--', 'Color', ...
            c(i, :));
    end
end
% Plot place matches
for robot_i = 1:N
    matches = decentr_state{robot_i}.place_matches;
    % Draw symmetric matches only once.
    for match_i = 1:numel(matches)
        if (numel(matches{match_i}) > 0)
            match = matches{match_i};
            from = robot_xz{robot_i}(match_i, :);
            to = robot_xz{match.robot_i}(match.frame_i, :);
            plot([from(1) to(1)], [from(2) to(2)], 'k-x', 'LineWidth', 5);
        end
    end
end
% Plot trajectories
for i = 1:numel(robot_xz)
    plot(robot_xz{i}(:, 1), robot_xz{i}(:, 2), '-', 'LineWidth', 3, ...
        'Color', c(i, :));
end
hold off;

axis equal;
end
