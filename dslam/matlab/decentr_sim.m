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

%% Set the following variables appropriately, manually, or load them.
%dataset_path
%dvpr_train_path
%sequence_id
%num_robots
%distributed_mapper_location <- -DCMAKE_BUILD_TYPE=Release !!!!
%data_type = 'kitti' or 'stata'
%parpool_size: Amount of worker threads for Gauss-Seidel, more means more
% can run in parallel, but also more RAM usage.
save('conf.mat', 'dataset_path', 'dvpr_train_path', 'sequence_id', ...
    'num_robots', 'distributed_mapper_location', 'data_type', ...
    'parpool_size');
%%
load('conf.mat');
%% Add to matlab path:
% * gtsam matlab toolbox
% * netvlad
% * export_fig from matlab file exchange

%% Preparation (do once, cached in files):
%% Generate NetVLAD vectors for dataset
getNetVladFeats(dataset_path, sequence_id, data_type);
%% Parse VO and merge with NetVLAD
% Prereq: Run C++ executable dataset_to_vo
parseAllData(dataset_path, sequence_id, data_type);
%% Split full dataset into n robots
overlap = 3;
if (strcmp(data_type, 'stata'))
    overlap = 10;
end
splitAndSave(dataset_path, sequence_id, num_robots, overlap);
%% Generate a live frame stream for in-order simulation.
generateDecentrFrameStream(dataset_path, sequence_id, num_robots);

%% Parameters
params.num_robots = num_robots;
params.netvlad_dim = 128;
params.clusters_per_robot = 1;
params.use_dvpr = 1;
% Minimum distance between geo verifications for a given pair of robots.
params.min_dist_geover = 10;
% Robust relative pose rejects outliers in relative pose detection.
params.robust_relpose_min_group_size = 2;
if (strcmp(data_type, 'kitti'))
    params.robust_relpose_consistency_range = 20;
    params.robust_relpose_position_tolerance = 4;
else
    assert(strcmp(data_type, 'stata'));
    params.robust_relpose_consistency_range = 10;
    params.robust_relpose_position_tolerance = 2;
end
params.opt_max_iters = 20;
params.use_tardioli = 1;
% For parameter studies
params.run_i = 1;
%% Actual simulation
root = [dataset_path '/' sequence_id '/'];
dpath = [root 'dslam/'];
%% Launch geometric verification server (once!)
% Launch the verification_request_server executable from pwd with args
% temp_request.txt temp_result.txt temp_lock.txt

%% Get stream and launch parpool.
load([dpath 'decentr_stream_' num2str(num_robots) '_robots.mat']);
load([dpath 'full_data_' num2str(num_robots) '_robots.mat']);
%%
parpool(parpool_size);
%% Run simulation and save
runSimulationAndSave(robot_vo_data, decentr_stream, ...
    dvpr_train_path, distributed_mapper_location, dpath, params, data_type);
%% Load results to plot
if (strcmp(data_type, 'stata'))
    load(regexprep([dpath 'run_data_' num2str(cell2mat(struct2cell(params))') '.mat'], '\s+', ' '));
else
    load([dpath 'run_data_' num2str(cell2mat(struct2cell(params))') '.mat']);
end
%% Plot final map state
figure(1);
plotDecentrState(decentr_state);
title('KITTI 00');
xlabel('x [m]');
ylabel('y [m]');
grid on;
%% Plot data transmission over time
figure(2);
plotDataOverTime(data_increments, data_increment_times, ...
    decentr_stream{end}.time);
%% Plot data transmission per robot pair
figure(3);
plotDataPerRobotPair(data_increments);
%% Plot accuracy stuff
figure(4);
orb_ate = 0;
plotAccuracy(accuracy_measurements, num_robots, ...
    decentr_stream{end}.time, orb_ate);
%% Figures to pdf
figure(1);
set(gcf, 'Position', [0 0 400 260]);
if ~exist('plots','dir'), mkdir('plots'); end
eval('export_fig plots/kitti.pdf -transparent -nocrop');
%%
figure(2);
set(gcf, 'Position', [0 0 400 260]);
eval('export_fig plots/data-time.pdf -transparent -nocrop');
%%
figure(3);
set(gcf, 'Position', [0 0 400 170]);
eval('export_fig plots/data-robots.pdf -transparent -nocrop');
%%
figure(4);
set(gcf, 'Position', [0 0 400 210]);
eval('export_fig plots/accuracy.pdf -transparent -nocrop');
%%
figure(5);
set(gcf, 'Position', [0 0 400 210]);
eval('export_fig plots/mdgv.pdf -transparent -nocrop');
%%
figure(6);
set(gcf, 'Position', [0 0 400 220]);
eval('export_fig plots/nvdim.pdf -transparent -nocrop');


%% BELOW: EXPERIMENTAL STUFF


%% Plot dgs stats
figure(5)
times = cellfun(@(x) x.end_time, dgs_stats);
num_iters = cellfun(@(x) x.num_iter, dgs_stats);
bytes = cellfun(@(x) x.exchange_gs, dgs_stats);

yyaxis left;
stem(times, num_iters);
ylim = get(gca, 'YLim');
hold on;
plot([decentr_stream{end}.time decentr_stream{end}.time], ylim, 'Color', 'red');
hold off;
title('Basic DGS stats');
xlabel('DGS finish time');
ylabel('Iteration count');
legend('DGS', 'Odometry end time', 'Location', 'NorthWest');

yyaxis right;
plot(times, bytes / 1e3, 'x');
ylabel('Data exchange [kB]');

%% Optimize using Dist Gauss Seidel
figure(4)
opt_state = runSyncGaussSeidel(...
    decentr_state, distributed_mapper_location, 1, 1:num_robots, 200);
plotDecentrState(decentr_state);
title('before');
figure(5)
plotDecentrState(opt_state);
title('after');
%% Plot quiver plot of all relative pose measurements
figure(5)
place_match_filter = ...
    cellfun(@(x) numel(x) > 0, decentr_state{1}.place_matches);
valid_place_matches = decentr_state{1}.place_matches(place_match_filter);
rel_poses = cellfun(@(x) x.Sim_M_Q, valid_place_matches, ...
    'UniformOutput', false);
all_quivers = posesToQuiver(rel_poses);
for i = 2:num_robots
    place_match_filter = ...
        cellfun(@(x) numel(x) > 0, decentr_state{i}.place_matches);
    valid_place_matches = decentr_state{i}.place_matches(place_match_filter);
    rel_poses = cellfun(@(x) x.Sim_M_Q, valid_place_matches, ...
        'UniformOutput', false);
    i_quivers = posesToQuiver(rel_poses);
    all_quivers = [all_quivers; i_quivers];
end
quiver3(all_quivers(:, 1), all_quivers(:, 2), all_quivers(:, 3), ...
    all_quivers(:, 4), all_quivers(:, 5), all_quivers(:, 6));
daspect([1 1 1]);
