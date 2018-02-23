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

function plotDataPerRobotPair(data_increments)

clf;

opt_increments = cellfun(@(x) x(:, :, 1), data_increments, ...
    'UniformOutput', false);
nv_increments = cellfun(@(x) x(:, :, 2), data_increments, ...
    'UniformOutput', false);
gv_increments = cellfun(@(x) x(:, :, 3), data_increments, ...
    'UniformOutput', false);
colormap hot;

assert(numel(opt_increments) > 0);
num_robots = size(opt_increments{1}, 1);

robot_traffics = ...
    [sum(cell2mat(reshape(opt_increments, 1, 1, [])) / 1e3, 3), ...
    zeros(num_robots, 1), ...
    sum(cell2mat(reshape(nv_increments, 1, 1, [])) / 1e3, 3), ...
    zeros(num_robots, 1), ...
    sum(cell2mat(reshape(gv_increments, 1, 1, [])) / 1e3, 3)];

imagesc(log10(robot_traffics));
line([num_robots + 1 num_robots + 1], [-0.5 num_robots + 1.5], ...
    'Color', 'white', 'LineWidth', 7);
line([(2 * num_robots) + 2 (2 * num_robots) + 2], ...
    [-0.5 num_robots + 1.5], 'Color', 'white', 'LineWidth', 7);
axis equal;
axis([0 (3 * num_robots) + 3 0 num_robots + 1]);
set(gca,'xtick',[],'ytick',[]);
set(gca,'XColor', 'white');
set(gca,'YColor', 'white');
colorbar;
ylabel('Sender', 'Color', 'k');
xlabel('Recipient', 'Color', 'k');
title(sprintf('log_{10}(transmitted data [kB])\nDOpt            DVPR           RelPose'));


end

