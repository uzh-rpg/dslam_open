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

function plotGlobalConfusion(robot_data)
%%
n_frames = cellfun(@(x) numel(x.T_W_C), robot_data);
offset = cumsum(n_frames);
global_n_frames = offset(end);
offset = [0; offset(1:end-1)];

%%
place_matches = [];
for robot_i = 1:numel(robot_data)
    for frame_i = 1:numel(robot_data{robot_i}.place_matches)
        if (numel(robot_data{robot_i}.place_matches{frame_i}) > 0)
            match = robot_data{robot_i}.place_matches{frame_i};
            place_matches = [place_matches [offset(robot_i) + frame_i;
                offset(match.robot_i) + match.frame_i]];
        end
    end
end

plot(place_matches(1, :), place_matches(2, :), '.');
axis([1 global_n_frames 1 global_n_frames]);
axis square

end

