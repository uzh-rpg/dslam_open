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

function [decentr_state, dgs_stats] = runSyncGaussSeidel(...
    decentr_state, distributed_mapper_location, group_idx, ...
    group_reindexing, max_iters)
outputDir = ['dgs_data/' num2str(group_idx)];
mkdir(outputDir);
writeDecentrStateToG2oFiles(decentr_state, outputDir, group_reindexing);

assert(system([distributed_mapper_location ' --dataDir ' pwd '/' ...
    outputDir '/ --nrRobots ' num2str(numel(decentr_state))...
    ' --traceFile ' pwd '/' outputDir '/trace --maxIter ' ...
    num2str(max_iters)]) == 0);

%% Retrieve optimization result
decentr_state = readDecentrStateFromOptG2oFiles(...
    outputDir, decentr_state, '_optimized');

%% Get stats
[dgs_stats.exchange_gs, dgs_stats.num_iter, dgs_stats.exchange_ddf, ...
    dgs_stats.comm_link_count] = getDistGaussSeidelStats(outputDir);

end

