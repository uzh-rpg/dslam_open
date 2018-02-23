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

%% Prep: Do once, writes to files
%% Generate NetVLAD vectors for dataset
getNetVladFeats(dataset_path, sequence_id);
%% Parse VO and merge with NetVLAD
% Prereq: Run C++ executable dataset_to_vo
parseAllData(dataset_path, sequence_id);
%% Split full dataset into n robots
splitAndSave(dataset_path, sequence_id, num_robots);
%% Generate verification requests.
generateVerificationRequests(dataset_path, sequence_id, num_robots);
%% Parse verification response
% Prereq: Run C++ exec process_verification_request
parseVerificationResponse(dataset_path, sequence_id, num_robots);



%% Load vanilla
root = [dataset_path '/' sequence_id '/'];
dpath = [root 'dslam/'];
load([dpath 'robot_data_' num2str(num_robots) '.mat']);
%% Plot vanilla
plotGeoverError(robot_data, [1], 'vanilla');

%% Create prototype
proto_data = robot_data;
%%
consistency_range = 20;
position_tolerance = 4;

for robot_i = 1:numel(proto_data)
    r_i_data = proto_data{robot_i};
    % For each robot, keep track of last seen rel pose that is floating or
    % accepted. Floating: rel pose that is neither accepted or rejected.
    last_floating = cell(num_robots, 1);
    last_accepted = cell(num_robots, 1);
    match_accepted = false(numel(r_i_data.place_matches), 1);
    for query_i = 1:numel(r_i_data.place_matches)
        match = r_i_data.place_matches{query_i};
        if (numel(match) > 0)
            this.frame_i = query_i;
            this.match = match;
            % Case there is a last accepted rel pose
            if (numel(last_accepted{match.robot_i}) > 0)
                assert(numel(last_floating{match.robot_i}) == 0);
                % If they are within range, accept if consistent.
                if areFramesWithinRange(r_i_data, ...
                        last_accepted{match.robot_i}.frame_i, query_i, ...
                        consistency_range)
                    if areMatchesConsistent(proto_data, robot_i, ...
                            last_accepted{match.robot_i}, this, ...
                            position_tolerance)
                        match_accepted(query_i) = true;
                        last_accepted{match.robot_i} = this;
                        disp('Acc consistent acc');
                        continue;
                    else
                        match_accepted(query_i) = false;
                        disp('Rej inconsistent acc');
                        continue;
                    end
                else % Frames are not within range: Forget last_accepted.
                    last_accepted{match.robot_i} = [];
                    % Will drop to case no previous floating.
                end
            end
            % Case there is a last floating rel pose
            if (numel(last_floating{match.robot_i}) > 0)
                % If they are within range, accept both if consistent.
                if areFramesWithinRange(r_i_data, ...
                        last_floating{match.robot_i}.frame_i, query_i, ...
                        consistency_range)
                    if areMatchesConsistent(proto_data, robot_i, ...
                            last_floating{match.robot_i}, this, ...
                            position_tolerance)
                        match_accepted(...
                            last_floating{match.robot_i}.frame_i) = true;
                        match_accepted(query_i) = true;
                        last_floating{match.robot_i} = [];
                        last_accepted{match.robot_i} = this;
                        disp('Acc consistent float');
                        continue;
                    % If not consistent, replace floating with this.
                    else
                        match_accepted(...
                            last_floating{match.robot_i}.frame_i) = false;
                        last_floating{match.robot_i} = this;
                        disp('Rej inconsistent float');
                        continue;
                    end
                else % Frames are not within range: Replace last_floating.
                    match_accepted(...
                        last_floating{match.robot_i}.frame_i) = false;
                    last_floating{match.robot_i} = this;
                    disp('Reinit float');
                    continue;
                end
            % Case there is neither a last accepted nor last floating.
            else
                % Set floating to this.
                last_floating{match.robot_i} = this;
                disp('Init float');
            end
        end
    end
    % Eliminate non-accepted matches.
    proto_data{robot_i}.place_matches(~match_accepted) = {[]};
    disp(['Accepted ' num2str(nnz(match_accepted)) ' of ' ...
        num2str(numel(match_accepted)) ' matches!']);
end

%%
plotGeoverError(proto_data, [2], 'robustified');
