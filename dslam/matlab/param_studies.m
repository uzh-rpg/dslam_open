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

% From decentr_sim.m load:
% robot_vo_data, decentr_stream, dvpr_train_path, 
% distributed_mapper_location, dpath, params
rerun = false;

%% Min dist geover parameter study: run.
params_to_vary = {'min_dist_geover', 'use_tardioli', 'run_i'};
runs = 1:3;
param_values = {[1 2 4 8 16 32 64], 0:1, runs};
%%
runParameterStudy(params_to_vary, param_values, robot_vo_data, ...
    decentr_stream, dvpr_train_path, distributed_mapper_location, ...
    dpath, params, rerun);
%% Min dist geover parameter study: load.
lines = getParamStudyLines(params_to_vary{1}, param_values{1}, runs, ...
    dpath, params);

%% Min dist geover parameter study: plot.

figure(5);
c = colormap('lines');
c = c([1 2 3 3 4], :);
style = {'-', '-', '-', '--', '-'};
div = [1e6 1e6 1e6 1e6 1];

clf;
hold on;
for line_i = 1:5
   linehand(line_i) = plot(param_values{1}, ...
       lines(line_i).avg / div(line_i), ...
       style{line_i}, 'Color', c(line_i, :));
   for x_i = 1:numel(param_values{1})
       if (numel(lines(line_i).result{x_i}) > 0)
           plot(param_values{1}(x_i), lines(line_i).result{x_i} / div(line_i), ...
               '.', 'Color', c(line_i, :));
      
       end
   end
end
hold off;

title('min dist gv parameter study');
xlabel('min dist. geo. verification [m]');
ylabel('total transmitted [MB]');
legend(linehand(1:5), 'DOpt', 'DVPR', 'RelPose', 'RelPose(desc)', ...
    'Final ATE [m]', 'Location', 'NorthEast');
set(gca, 'XScale', 'log');
set(gca, 'YLim', [0 13]);
set(gca, 'XTick', param_values{1});
grid on;


%% NVDIM parameter study: run.
params_to_vary = {'netvlad_dim', 'use_tardioli', 'run_i'};
runs = 1:3;
param_values = {64:16:160, 0:1, runs};
%%
runParameterStudy(params_to_vary, param_values, robot_vo_data, ...
    decentr_stream, dvpr_train_path, distributed_mapper_location, ...
    dpath, params, rerun);
%% NVDIM parameter study: load.
lines = getParamStudyLines(params_to_vary{1}, param_values{1}, runs, ...
    dpath, params);

%% Min dist geover parameter study: plot.

figure(6);
c = colormap('lines');
c = c([1 2 3 3 5 4 4], :);
style = {'-', '-', '-', '--', '-', '-', '--'};
div = [1e6 1e6 1e6 1e6 1 1e6 1e6];

clf;
hold on;
for line_i = 1:5
   linehand(line_i) = plot(param_values{1}, ...
       lines(line_i).avg / div(line_i), ...
       style{line_i}, 'Color', c(line_i, :));
   for x_i = 1:numel(param_values{1})
       if (numel(lines(line_i).result{x_i}) > 0)
           plot(param_values{1}(x_i), lines(line_i).result{x_i} / div(line_i), ...
               '.', 'Color', c(line_i, :));
      
       end
   end
end
for line_i = 6:7
    linehand(line_i) = plot(param_values{1}, ...
        (lines(2).avg + lines(line_i - 3).avg) / div(line_i), ...
        style{line_i}, 'Color', c(line_i, :));
end
hold off;

title('NetVLAD dimension parameter study');
xlabel('NetVLAD dimensions');
ylabel('total transmitted [MB]');
legend(linehand([1 2 3 6 5]), 'DOpt', 'DVPR', 'RelPose', ...
    'DVPR+RelPose', 'Final ATE [m]', 'Location', 'NorthEast');
set(gca, 'YLim', [0 13]);
set(gca, 'XLim', [64 160]);
set(gca, 'XTick', param_values{1});
grid on;
