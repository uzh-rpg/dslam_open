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

function [decentr_state, dgs_stats, final_increments, ...
    final_increment_times, opt_handle, final_accuracy_measurements] = ...
    wrapUpOptimization(...
    decentr_state, opt_handle, distributed_mapper_location, params, ...
    final_time)

t = tic;
% Collect data from still running optimization and launch last one.
[decentr_state, opt_handle, dgs_stats, opt_increment] = ...
    manageAsyncGaussSeidel(...
    decentr_state, opt_handle, distributed_mapper_location, true, ...
    params.opt_max_iters, true);
t0 = toc(t);
for i = 1:numel(dgs_stats)
    dgs_stats{i}.end_time = final_time + t0;
end
final_increments = ...
    {cat(3, opt_increment, zeros(size(opt_increment)), ...
    zeros(size(opt_increment)))};
final_increment_times = [final_time + t0];
[accuracy_measurement, decentr_state] = ...
    evalAccuracy(...
    decentr_state, opt_handle, final_time + t0);
final_accuracy_measurements = {accuracy_measurement};
plotDecentrState(decentr_state);
pause(0.0001);

% Collect last one (final launch will be ignored).
[decentr_state, ~, last_dgs_stats, opt_increment] = ...
    manageAsyncGaussSeidel(...
    decentr_state, opt_handle, distributed_mapper_location, true, ...
    params.opt_max_iters, false);
t1 = toc(t);
for i = 1:numel(last_dgs_stats)
    last_dgs_stats{i}.end_time = final_time + t1;
end
dgs_stats = [dgs_stats; last_dgs_stats];
final_increments = [final_increments;
    cat(3, opt_increment, zeros(size(opt_increment)), ...
    zeros(size(opt_increment)))];
final_increment_times = [final_increment_times; final_time + t1];
[accuracy_measurement, decentr_state] = ...
    evalAccuracy(decentr_state, opt_handle, final_time + t1);
final_accuracy_measurements = [final_accuracy_measurements; ...
    accuracy_measurement];
plotDecentrState(decentr_state);
pause(0.0001);

end

