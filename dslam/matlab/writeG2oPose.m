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

function writeG2oPose(file_id, robot_idx, frame_idx, T_W_C)

frame_id = gtsamFrameId(robot_idx, frame_idx);


x = T_W_C(1, 4); y = T_W_C(2, 4); z = T_W_C(3, 4);
R = T_W_C(1:3, 1:3);
assert(sum(sum(imag(R))) == 0);
% Seems to be necessary to avoid complex quaternions
% (to avoid trace(R) < -1)
R = R / (det(R) + 1e-5);
q = rot2quat(R, 1e-4);
%quat2rot(q) - R
%assert(all(all(quat2rot(q) - R < 1e-5)));
assert(norm(q)>1e-3);
q = q/norm(q);
assert(norm(q)>1e-3);
qw = q(1); qx = q(2); qy = q(3); qz = q(4);
if (sum(imag(q)) ~= 0)
    q
    assert(false)
end

fprintf(file_id, ...
    'VERTEX_SE3:QUAT %d %f %f %f %f %f %f %f\n', ...
    frame_id, x, y, z, qx, qy, qz, qw);

end

