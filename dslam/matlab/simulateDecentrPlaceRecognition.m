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

function [decentr_state, dvpr_increment, gv_increment, ...
    netvlad_match_stats] = ...
    simulateDecentrPlaceRecognition(decentr_state, robot_i, params)

N = numel(decentr_state);
dvpr_increment = zeros(N);
gv_increment = zeros(N);

%% 
query_netvlad = decentr_state{robot_i}.netvlad(:, end);
query_info = whos('query_netvlad');
query_frame_i = numel(decentr_state{robot_i}.Sim_O_C);

% Matches to own past:
time_dist = 30;
candidates = decentr_state{robot_i}.times < ...
    (decentr_state{robot_i}.times(end) - time_dist);
[distance, best_frame] = ...
    pdist2(decentr_state{robot_i}.netvlad(:, candidates)', ...
    query_netvlad', 'squaredeuclidean', 'Smallest', 1);
if (distance < 0.01)
    [decentr_state, ~] = ...
        simulateDecentrVerification(...
        decentr_state, robot_i, best_frame, ...
        robot_i, query_frame_i, params);
end

% Matches to other robots:
if params.use_dvpr
    % Find assigned robot.
    cluster_centers = cellfun(@(x) x.cluster_center', decentr_state, ...
        'UniformOutput', false);
    cluster_centers = cell2mat(cluster_centers);
    [~, cluster_robot] = pdist2(cluster_centers, query_netvlad', ...
        'squaredeuclidean', 'Smallest', 1);
    
    % Query so far existing entries.
    if (numel(decentr_state{cluster_robot}.dvpr_queries_netvlad) > 0)
        [min_dist, min_dist_index] = ...
            pdist2(decentr_state{cluster_robot}.dvpr_queries_netvlad, ...
            query_netvlad', 'squaredeuclidean', 'Smallest', 1);
        match_robot_i = ...
            decentr_state{cluster_robot}.dvpr_queries_robot_i(...
            min_dist_index);
        match_frame_i = ...
            decentr_state{cluster_robot}.dvpr_queries_frame_i(...
            min_dist_index);
        
        % For now, ignore self-matches (internal loop closures)
        if (match_robot_i == robot_i)
            min_dist = Inf;
            match_robot_i = 0;
            match_frame_i = 0;
        end
    else
        min_dist = Inf;
        match_robot_i = 0;
        match_frame_i = 0;
    end
    
    % Add new entry due to query.
    decentr_state{cluster_robot}.dvpr_queries_netvlad = [...
        decentr_state{cluster_robot}.dvpr_queries_netvlad; ...
        query_netvlad'];
    decentr_state{cluster_robot}.dvpr_queries_robot_i = [...
        decentr_state{cluster_robot}.dvpr_queries_robot_i; ...
        robot_i];
    decentr_state{cluster_robot}.dvpr_queries_frame_i = [...
        decentr_state{cluster_robot}.dvpr_queries_frame_i; ...
        query_frame_i];
    
    % Log data due to place recognition
    if cluster_robot ~= robot_i
        % query size + 1 byte robot id + 4 bytes frame id
        dvpr_increment(robot_i, cluster_robot) = ...
            dvpr_increment(robot_i, cluster_robot) + query_info.bytes + 5;
        % 8 bytes double distance + 4 bytes frame id + 1 byte robot id
        dvpr_increment(cluster_robot, robot_i) = ...
            dvpr_increment(cluster_robot, robot_i) + 13;
    end
else  % Query-all in broad phase.
    distances = inf(1, N);
    dist_info = whos('distances');
    best_frames = inf(1, N);
    frame_info = whos('best_frames');
    for queried_robot = 1:N
        if queried_robot == robot_i
            continue
        end
        dvpr_increment(robot_i, queried_robot) = ...
            dvpr_increment(robot_i, queried_robot) + query_info.bytes;

        if (numel(decentr_state{queried_robot}.netvlad) > 0)
            [distances(queried_robot), best_frames(queried_robot)] = ...
                pdist2(decentr_state{queried_robot}.netvlad', query_netvlad', ...
                'squaredeuclidean', 'Smallest', 1);
        end
        % 8 bytes double distance + 4 bytes frame id
        dvpr_increment(queried_robot, robot_i) = ...
            dvpr_increment(queried_robot, robot_i) + 8;
    end
    [min_dist, match_robot_i] = min(distances);
    match_frame_i = best_frames(match_robot_i);
end

%% Enforce min distance between geometric verifications:
if params.min_dist_geover > 0
    % Find last frame matched to same robot
    matched_frames = find(cellfun(...
        @(x) numel(x) > 0, decentr_state{robot_i}.place_matches));
    same_pair = find(cellfun(@(x) x.robot_i == match_robot_i, ...
        decentr_state{robot_i}.place_matches(matched_frames)));
    if (numel(same_pair) > 0)
        recent = matched_frames(same_pair(end));
        if norm(decentr_state{robot_i}.Sim_O_C{end}(1:3, 4) - ...
                decentr_state{robot_i}.Sim_O_C{recent}(1:3, 4)) < ...
                params.min_dist_geover
            netvlad_match_stats = zeros(1, 4);
            disp('Discarding place recognition near previous match!');
            return;
        end
    end
end

%% From all distance responses, select the smallest & gv if below thresh
if (min_dist < 0.01)
   [decentr_state, gv_increment] = ...
       simulateDecentrVerification(...
       decentr_state, match_robot_i, match_frame_i, ...
       robot_i, query_frame_i, params);
   netvlad_match_stats = ...
       [robot_i query_frame_i match_robot_i match_frame_i];
else
    netvlad_match_stats = zeros(1, 4);
end

end

