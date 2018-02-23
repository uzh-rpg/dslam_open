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

function [decentr_state, gv_increment] = ...
    simulateDecentrVerification(...
    decentr_state, match_robot_i, match_frame_i, ...
    query_robot_i, query_frame_i, params)

N = numel(decentr_state);
gv_increment = zeros(N);

verification_request = makeVerificationRequest(...
    decentr_state, match_robot_i, match_frame_i, ...
    query_robot_i, query_frame_i, params);

% Calculate data transmitted per geometric verification:
% 1 byte query robot id
% 4 bytes query frame id
% 4 bytes match frame id
% #(query_kp)*32 bytes keypoint descriptors OR
% #(query_kp)*2 bytes word ids
if params.use_tardioli
    kp_description = 2;
else
    kp_description = 32;
end
% #(query_kp)*3*4 bytes landmark positions (3 singles)
n_query_kp = nnz(verification_request(:, 3) == 0);
gv_increment(query_robot_i, match_robot_i) = ...
    9 + n_query_kp * (kp_description + 3*4);

fid = fopen('temp_lock.txt', 'w');
dlmwrite('temp_request.txt', verification_request, 'delimiter', ' ', ...
    'precision', 10);
fclose(fid);
delete('temp_lock.txt');

while exist('temp_result.txt', 'file') == 0
    pause(0.001);
end
while exist('temp_lock.txt', 'file') ~= 0
    pause(0.001);
end

verification_result = load('temp_result.txt');
assert(system('rm temp_result.txt') == 0);

% Response: 6*8 bytes pose, 6*8 bytes consistency check pose
gv_increment(match_robot_i, query_robot_i) = 2 * 6 * 8;

if verification_result(5) == 0
    return
end

Sim_M_Q = reshape(verification_result(6:end), [4 4]);

% Build match struct
match.robot_i = match_robot_i;
match.frame_i = match_frame_i;
match.Sim_M_Q = Sim_M_Q;

match_accepted = true;
% Check consistency if option toggled
if params.robust_relpose_min_group_size > 1
    [match_accepted, ...
        decentr_state{query_robot_i}.consistent_groups{match.robot_i}] = ...
        acceptMatchIfConsistent(match, query_frame_i, query_robot_i, ...
        decentr_state, params);
end

% Save place recognition.
if (match_accepted)
    decentr_state{query_robot_i}.place_matches{query_frame_i} = match;
    % Symmetry!
    decentr_state{match_robot_i}.place_matches{match_frame_i}.robot_i = query_robot_i;
    decentr_state{match_robot_i}.place_matches{match_frame_i}.frame_i = query_frame_i;
    decentr_state{match_robot_i}.place_matches{match_frame_i}.Sim_M_Q = Sim_M_Q ^ -1;
    
    decentr_state{query_robot_i}.converged = false;
    decentr_state{match_robot_i}.converged = false;
    
    % Handle groups.
    match_group = ...
        [decentr_state{match_robot_i}.grouped_with match_robot_i];
    query_group = ...
        [decentr_state{query_robot_i}.grouped_with query_robot_i];
    if (~any(match_group == query_robot_i))
        for grouped_robot = match_group
            decentr_state{grouped_robot}.grouped_with = unique([...
                decentr_state{grouped_robot}.grouped_with query_group]);
        end
        for grouped_robot = query_group
            decentr_state{grouped_robot}.grouped_with = unique([...
                decentr_state{grouped_robot}.grouped_with match_group]);
        end
    end
    
    assert(~any(...
        decentr_state{query_robot_i}.grouped_with == query_robot_i));
    assert(~any(...
        decentr_state{match_robot_i}.grouped_with == match_robot_i));
end

end
