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

function T = rpyxyzToT(rpyxyz)
% From roll, pitch, yaw, x, y, z to the corresponding matrix.
% R = R_roll * R_pitch * R_yaw, i.e. order of application is ypr
% x looks to the front, i.e. roll is around x.
% All rotations are applied with the right-hand rule.

T = eye(4);

assert(numel(rpyxyz) == 6);
roll = rpyxyz(1);
pitch = rpyxyz(2);
yaw = rpyxyz(3);

R_roll = [
    1 0         0
    0 cos(roll) -sin(roll)
    0 sin(roll) cos(roll)];
R_pitch = [
    cos(pitch)  0 sin(pitch)
    0           1 0
    -sin(pitch) 0 cos(pitch)];
R_yaw = [
    cos(yaw) -sin(yaw) 0
    sin(yaw) cos(yaw)  0
    0        0         1];

R = R_roll * R_pitch * R_yaw;

T(1:3, 1:3) = R;
T(1:3, 4) = rpyxyz(4:end);

end

