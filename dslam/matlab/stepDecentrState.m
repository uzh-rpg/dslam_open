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

function ...
    [decentr_state, data_increment, netvlad_match_stats, opt_future, ...
    dgs_stats] = ...
    stepDecentrState(decentr_state, stream_data, params, ...
    distributed_mapper_location, opt_future)
% data_increment: nxnx3, where the 3 layers represent data transfer due to
% optimization, broad-phase place recognition and geometric verification.
% In each layer the (i,j)-th element represents transfer from i to j.

dgs_stats = {};

%% Append new data to state
robot_i = stream_data.robot_i;
if numel(decentr_state{robot_i}.Sim_O_C) > 0
    decentr_state{robot_i}.Sim_O_C = ...
        [decentr_state{robot_i}.Sim_O_C; ...
        decentr_state{robot_i}.Sim_O_C{end} * stream_data.Sim_Cprev_C];
    decentr_state{robot_i}.Sim_O_C{end}(1:3, 1:3) = ...
        fixR(decentr_state{robot_i}.Sim_O_C{end}(1:3, 1:3));
    assert((det(decentr_state{robot_i}.Sim_O_C{end}(1:3, 1:3))...
            - 1) < 1e-5);
    decentr_state{robot_i}.Sim_Cprev_C = ...
        [decentr_state{robot_i}.Sim_Cprev_C; stream_data.Sim_Cprev_C];
else
    decentr_state{robot_i}.Sim_O_C = ...
        [decentr_state{robot_i}.Sim_O_C; eye(4)];
end
decentr_state{robot_i}.original_T_O_C = ...
    [decentr_state{robot_i}.original_T_O_C; stream_data.original_T_O_C];
decentr_state{robot_i}.times = ...
    [decentr_state{robot_i}.times; stream_data.time];
decentr_state{robot_i}.netvlad = ...
    [decentr_state{robot_i}.netvlad stream_data.netvlad(...
    1:params.netvlad_dim, :)];
decentr_state{robot_i}.descs = ...
    [decentr_state{robot_i}.descs; stream_data.descs];
decentr_state{robot_i}.wids3 = ...
    [decentr_state{robot_i}.wids3; stream_data.wids3];
decentr_state{robot_i}.wids4 = ...
    [decentr_state{robot_i}.wids4; stream_data.wids4];
decentr_state{robot_i}.lms_in_frame = ...
    [decentr_state{robot_i}.lms_in_frame; stream_data.lms_in_frame];

decentr_state{robot_i}.gt_T_W_C = ...
    [decentr_state{robot_i}.gt_T_W_C; stream_data.gt_T_W_C];

%% Perform decentralized place recognition
[decentr_state, dvpr_increment, gv_increment, netvlad_match_stats] = ...
    simulateDecentrPlaceRecognition(decentr_state, robot_i, params);

%% Perform some decentralized optimization
opt_increment = zeros(params.num_robots);
[decentr_state, opt_future, dgs_stats, opt_increment] = ...
    manageAsyncGaussSeidel(decentr_state, ...
    opt_future, distributed_mapper_location, false, ...
    params.opt_max_iters, true);

for i = 1:numel(dgs_stats)
    dgs_stats{i}.end_time = stream_data.time;
end

data_increment = cat(3, opt_increment, dvpr_increment, gv_increment);

end

