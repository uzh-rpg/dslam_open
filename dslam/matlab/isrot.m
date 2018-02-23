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

function x = isrot (a, tol)
% Use: bool = isrot(R) (counterintuitive: return 0 if it's a rotation)
% Checks if the 3x3 matrix R is a rotation matrix
%      isrot(R)==0 rotation matrix
%      isrot(R)~=0 not a rotation
%
% From Luca Carlone's
% https://bitbucket.org/lucacarlone/pgo3d-duality-opencode/src/ebb6e1b8cebaad7f2aaf581b1d0c0bad737faebb/lib/isrot.m?at=master&fileviewer=file-view-default

if nargin < 2
    tol = 1e-5;
end

[r c] = size(a);
x = 1;
if r ~= 3
    disp ('Matrix has not 3 rows')
elseif c ~= 3
    disp ('Matrix has not 3 columns')
elseif norm ( a * a' - eye(3) ) > tol
    disp ('Matrix is not orthonormal, i.e. ||(R''R-I)|| > 1E-10')
    disp(norm ( a * a' - eye(3) ))
    disp('-----------------------')
elseif det (a) < 0
    disp ('Matrix determinant is -1')
else x = 0;
end
