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

% Visualize trajectory outputs
import gtsam.*
close all;
nr_robots = 10;
dataDir = 'data';
cmap = hsv(nr_robots);


    figure(1);
    cla; hold on;
for robot_i = 1:nr_robots
    robot_i
    initial_file = sprintf('%s/%d_optimized.g2o', dataDir, robot_i-1)
    [graph, initial] = gtsam.readG2o(initial_file, true);
    
    plot3DSubGraph(graph, initial, '-',  cmap(robot_i,:), '');
    hold on;
end

axis equal;
    
