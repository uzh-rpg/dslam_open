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

function runSimulationAndSave(robot_vo_data, decentr_stream, ...
    dvpr_train_path, distributed_mapper_location, dpath, params, data_type)
%% Prepare empty decentralized state.
decentr_state = initDecentrState(robot_vo_data);
%% Assign netvlad cluster centers
% set dvpr_train_path to folder containing a bunch of features
decentr_state = assignClusterCenters(...
    decentr_state, dvpr_train_path, params);

%% Play!
figure(1);
tic;
netvlad_match_stats = zeros(numel(decentr_stream), 4);
data_increments = cell(numel(decentr_stream), 1);
accuracy_measurements = {};
opt_handle = initOptHandle(params.num_robots);
dgs_stats = {};
% Start ATE eval 5 seconds into the dataset to avoid meaningless aligns.
last_accur_eval = 5;
for step_i = 1:numel(decentr_stream)
%for step_i = 1:500
    real_time = toc
    if (real_time < decentr_stream{step_i}.time)
        pause(decentr_stream{step_i}.time - real_time);
    end
    [decentr_state, data_increments{step_i}, ...
        netvlad_match_stats(step_i, :), opt_handle, dgs_stats_i] = ...
        stepDecentrState(decentr_state, decentr_stream{step_i}, params, ...
        distributed_mapper_location, opt_handle);
    dgs_stats = [dgs_stats; dgs_stats_i];
    
    if (mod(step_i, params.num_robots) == 0)
        plotDecentrState(decentr_state);
        pause(0.0001);
    end
    
    if ((decentr_stream{step_i}.time - last_accur_eval) > 1)
        last_accur_eval = decentr_stream{step_i}.time;
        [accuracy_measurement, decentr_state] = ...
            evalAccuracy(decentr_state, opt_handle, ...
            decentr_stream{step_i}.time);
        accuracy_measurements = [accuracy_measurements; ...
            accuracy_measurement];
    end
    disp([num2str(step_i) ' out of ' num2str(numel(decentr_stream))]);
end
data_increment_times = cellfun(@(x) x.time, decentr_stream);
% Wrap optimization, unless optimization is nop.
vo_end_state = decentr_state;
[decentr_state, dgs_stats_i, final_increments, ...
    final_increment_times, opt_handle, final_accuracy_measurements] = ...
    wrapUpOptimization(...
    decentr_state, opt_handle, ...
    distributed_mapper_location, params, decentr_stream{end}.time);

dgs_stats = [dgs_stats; dgs_stats_i];
data_increments = [data_increments; final_increments];
data_increment_times = [data_increment_times; final_increment_times];
accuracy_measurements = [...
    accuracy_measurements; final_accuracy_measurements];
plotDecentrState(decentr_state);
%% Save stats
%dlmwrite(...
%    [dpath 'netvlad_match_stats_' ...
%    num2str(cell2mat(struct2cell(params))') '.txt'], ...
%    netvlad_match_stats, ' ');
if strcmp(data_type, 'stata')
    save(...
        regexprep([dpath 'run_data_' ...
        num2str(cell2mat(struct2cell(params))') '.mat'], '\s+', ' '), ...
        'decentr_state', 'data_increments', 'data_increment_times', ...
        'accuracy_measurements', 'dgs_stats', 'vo_end_state', '-v7.3');
else
    save(...
        [dpath 'run_data_' ...
        num2str(cell2mat(struct2cell(params))') '.mat'], ...
        'decentr_state', 'data_increments', 'data_increment_times', ...
        'accuracy_measurements', 'dgs_stats', '-v7.3');

end

