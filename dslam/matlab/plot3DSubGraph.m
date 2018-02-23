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

function plot3DSubGraph(graph, values, linespec, colorspec, markerspec)
%PLOTGRAPH Plots the Pose3's in a values, with optional covariances
%   Finds all the Pose2 objects in the given Values object and plots the graph.
% If a Marginals object is given, this function will also plot marginal
% covariance ellipses for each pose.

import gtsam.*


if ~exist('linespec', 'var') || isempty(linespec)
    linespec = '-';
end


if ~exist('colorspec', 'var') || isempty(colorspec)
    colorspec = 'r';
end


if ~exist('markerspec', 'var') || isempty(markerspec)
    markerspec = '';
end


holdstate = ishold;
hold on


L = {linespec};
% Draw graph
for i = 0:graph.size - 1
    factor = graph.at(i);
    if size(factor.keys) == 2
        key1 = factor.keys.at(0);
        key2 = factor.keys.at(1);

        try
        pose1 = values.atPose3(key1);
        pose2 = values.atPose3(key2);
        % draw a line - do not show loop closures if showLoopClosures is
        % false                
        key1_sym = symbolChr(key1);
        key2_sym = symbolChr(key2);
        key1_index = symbolIndex(key1);
        key2_index = symbolIndex(key2);

        if key1_sym ~= key2_sym
            plot([pose1.x, pose2.x], [pose1.z, pose2.z], 'Linestyle', '-', 'Color', 'k', 'Marker', markerspec, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r', 'alpha', 0.5)
        else
            plot([pose1.x, pose2.x], [pose1.z, pose2.z], 'Linestyle', '-o', 'Color', colorspec)
        end

        catch
        end
    end
end


if ~holdstate
    hold off
end

end
