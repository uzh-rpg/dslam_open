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

function verification_request = makeVerificationRequest(...
    decentr_state, match_robot_i, match_frame_i, ...
    query_robot_i, query_frame_i, params)

verification_request = [];

% Cat query
original_T_C_O = ...
    decentr_state{query_robot_i}.original_T_O_C{query_frame_i} ^ -1;
p_O_lm = decentr_state{query_robot_i}.p_O_lm;
p_O_lm = p_O_lm(...
    :, decentr_state{query_robot_i}.lms_in_frame{query_frame_i});
p_C_lm = (original_T_C_O(1:3, 1:3) * p_O_lm + original_T_C_O(1:3, 4))';
if params.use_tardioli
    descs = decentr_state{query_robot_i}.wids4{query_frame_i};
else
    descs = decentr_state{query_robot_i}.descs{query_frame_i};
end
num_kp = size(descs, 1);
verification_request = [verification_request; ...
    repmat([query_robot_i query_frame_i 0], [num_kp, 1]) single(descs) p_C_lm];

% Cat match
original_T_C_O = ...
    decentr_state{match_robot_i}.original_T_O_C{match_frame_i} ^ -1;
p_O_lm = decentr_state{match_robot_i}.p_O_lm;
p_O_lm = p_O_lm(...
    :, decentr_state{match_robot_i}.lms_in_frame{match_frame_i});
p_C_lm = (original_T_C_O(1:3, 1:3) * p_O_lm + original_T_C_O(1:3, 4))';
if params.use_tardioli
    descs = decentr_state{match_robot_i}.wids4{match_frame_i};
else
    descs = decentr_state{match_robot_i}.descs{match_frame_i};
end
num_kp = size(descs, 1);
verification_request = [verification_request; ...
    repmat([match_robot_i match_frame_i 1], [num_kp, 1]) single(descs) p_C_lm];

end

