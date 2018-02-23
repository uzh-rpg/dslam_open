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

function [decentr_state, opt_handle, dgs_stats, opt_increment] = ...
    manageAsyncGaussSeidel(...
    decentr_state, opt_handle, distributed_mapper_location, ...
    wait_finished, max_iters, renew)

n_robots = numel(decentr_state);
dgs_stats = {};
opt_increment = zeros(n_robots);

% If an optimization has previously been launched...
if (opt_handle.running)
    % If optimization still running, don't do anything.
    for i = 1:numel(opt_handle.futures)
        if (~isempty(opt_handle.futures{i}))
            if (wait_finished)
                wait(opt_handle.futures{i});
            else
                if (~strcmp(opt_handle.futures{i}.State, 'finished'))
                    return;
                end
            end
        end
    end
    % Catch any failures.
    for i = 1:numel(opt_handle.futures)
        if (~isempty(opt_handle.futures{i}))
            if (~isempty(opt_handle.futures{i}.Error))
                opt_handle.futures{i}.Error
                opt_handle.futures{i}.Diary
                opt_handle.futures{i}.Error.stack(1)
                opt_handle.futures{i}.Error.stack(end)
                assert(false);
            end
        end
    end
    
    % Optimization finished successfully: Get stats, update state.
    update_state = cell(n_robots, 1);
    for i = 1:numel(opt_handle.futures)
        if (~isempty(opt_handle.futures{i}))
            update_state(opt_handle.group_assignment == i) = ...
                opt_handle.futures{i}.OutputArguments{1};
            dgs_stats_i = opt_handle.futures{i}.OutputArguments{2};
            dgs_stats = [dgs_stats; dgs_stats_i];
            
            % Reconstruct communication between pairs of robots.
            dgs_stats_i.comm_link_count
            opt_handle.between_robot_links{i}
            links = opt_handle.between_robot_links{i};
            links = links + links';
            linksum = sum(links(:));
            assert(dgs_stats_i.comm_link_count == linksum);
            
            group = find(opt_handle.group_assignment == i);
            group_data_exchange = ...
                links * dgs_stats_i.exchange_gs / linksum;
            opt_increment(group, group) = ...
                opt_increment(group, group) + group_data_exchange;
        end
    end
    
    decentr_state = consistentlyApplyState(update_state, decentr_state);
    decentr_state = setConvergedWhereApplicable(...
        update_state, decentr_state);
    
    % Indicate which groups are co-optimized, for accuracy eval:
    opt_handle.optimized_groups = [];
    groups = unique(opt_handle.group_assignment);
    for group_i = 1:numel(groups)
        opt_handle.optimized_groups(group_i).members = find(...
            opt_handle.group_assignment == groups(group_i));
    end
    disp (['Have ' num2str(numel(opt_handle.optimized_groups)) ...
        ' optimization groups.']);
end

if renew
    % Launch new optimization
    %opt_future = parfeval(gcp(), @runSyncGaussSeidel, 1, decentr_state, ...
    %    distributed_mapper_location);

    opt_handle.group_assignment = 1:n_robots;
    for i = 1:n_robots
        opt_handle.group_assignment(i) = ...
            min([decentr_state{i}.grouped_with i]);
    end

    opt_groups = unique(opt_handle.group_assignment);

    opt_handle.futures = {};
    for group_i = opt_groups
        group_mask = (opt_handle.group_assignment == group_i);
        group_size = nnz(group_mask);
        if group_size > 1
            members_converged = cellfun(@(x) x.converged, ...
                decentr_state(group_mask));
            if ~all(members_converged)
                group_reindexing = zeros(n_robots, 1);
                group_reindexing(group_mask) = 1:group_size;
                opt_handle.futures{group_i} = parfeval(gcp(), ...
                    @runSyncGaussSeidel, 2, decentr_state(group_mask), ...
                    distributed_mapper_location, group_i, group_reindexing, ...
                    max_iters);
                opt_handle.between_robot_links{group_i} = ...
                    countBetweenRobotLinks(decentr_state(group_mask), ...
                    group_reindexing);
            end
        end
    end
    opt_handle.running = true;
end

end

