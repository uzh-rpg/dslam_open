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

function result = areMatchesConsistent(all_robot_data, query_robot, ...
    match_a, match_b, position_tolerance)
% Basically make sure that 
% T_M1_Q1 * T_Q1_Q2 is similar to T_M1_M2 * T_M2_Q2, i.e.
% that they are within tolerance.
T_M1_Q1 = match_a.match.Sim_M_Q;
T_M2_Q2 = match_b.match.Sim_M_Q;
% T_Q1_Q2
T_O_Q1 = all_robot_data{query_robot}.Sim_O_C{match_a.frame_i};
T_O_Q2 = all_robot_data{query_robot}.Sim_O_C{match_b.frame_i};
T_Q1_Q2 = tInv(T_O_Q1) * T_O_Q2;
% T_M1_M2
match_robot = match_a.match.robot_i;
assert(match_robot == match_b.match.robot_i);
T_O_M1 = all_robot_data{match_robot}.Sim_O_C{match_a.match.frame_i};
T_O_M2 = all_robot_data{match_robot}.Sim_O_C{match_b.match.frame_i};
T_M1_M2 = tInv(T_O_M1) * T_O_M2;

T_M1_Q2_1 = T_M1_Q1 * T_Q1_Q2;
T_M1_Q2_2 = T_M1_M2 * T_M2_Q2;

error = tInv(T_M1_Q2_1) * T_M1_Q2_2;

result = norm(error(1:3, 4)) < position_tolerance;

end

