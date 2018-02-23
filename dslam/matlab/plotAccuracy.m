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

function plotAccuracy(accuracy_measurements, num_robots, vo_end_time, ...
    orb_ate)

times = cellfun(@(x) x.time, accuracy_measurements);
num_meas = numel(accuracy_measurements);

robot_accuracies = cell(num_robots, 1);
mg_sizes = cell(num_robots, 1);
og_sizes = cell(num_robots, 1);
for robot_i = 1:num_robots
    robot_accuracies{robot_i} = zeros(num_meas, 1);
    og_sizes{robot_i} = zeros(num_meas, 1);
    mg_sizes{robot_i} = zeros(num_meas, 1);
    for meas_i = 1:num_meas
        meas = accuracy_measurements{meas_i};
        for mg_i = 1:numel(meas.matched_groups)
            if (any(meas.matched_groups(mg_i).members == robot_i))
                mg_sizes{robot_i}(meas_i) = ...
                    meas.matched_groups(mg_i).num_frames;
                break;
            end
        end
        for og_i = 1:numel(meas.optimized_groups)
            if (any(meas.optimized_groups(og_i).members == robot_i))
                robot_accuracies{robot_i}(meas_i) = ...
                    meas.optimized_groups(og_i).ATE;
                og_sizes{robot_i}(meas_i) = ...
                    meas.optimized_groups(og_i).num_frames;
                break;
            end
        end
    end
end
yyaxis left;
ate = plot(times, robot_accuracies{1}, '-');
hold on;
for i = 2:num_robots
    plot(times, robot_accuracies{i}, '-');
end
%orb = plot(xlim, [orb_ate orb_ate], 'b--');
hold off;
ylabel('ATE [m]');
xlabel('time [s]');
grid on;

yyaxis right;
opted = plot(times, og_sizes{1}, '-');
hold on;
for i = 2:num_robots
    plot(times, og_sizes{i}, '-');
end
%for i = 1:num_robots
%    matched = plot(times, mg_sizes{i}, '.');
%end
et = plot([vo_end_time vo_end_time], ylim, 'k--');
hold off;
ylabel('frame count in CC');
title('accuracy development');
set(gca, 'XLim',[min(times) max(times)]);
%legend([ate, opted, matched, et, orb], 'ATE', 'optimized', 'matched', ...
%    'VO end time', 'ORB SLAM','Location', 'NorthWest');
legend(et, 'VO end time', 'Location', 'NorthWest');

end

