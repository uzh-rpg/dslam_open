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

function [informationExchangeGaussSeidel, nrGaussSeidelIterations, ...
    informationExchangeDDF, commLinks] = getDistGaussSeidelStats(dir)

filename = [dir '/trace'];
trace_file = sprintf('%s_overall_error.txt',filename);
trace = dlmread(trace_file, ' ');

end_index = find(trace(1,:) == -1);
rotation_iterations = end_index-1;

end_index = find(trace(2,:) == -1);
pose_iterations = end_index-1;

centralized_error = trace(3,1);
distributed_error = trace(4,1);

sizePose = 6*8;  % Assumed to be 6 doubles, 1 double = 8 byte
sizeRotation = 9*8;  % Assumed to be 9 doubles, 1 double = 8 byte

commLinks = 2 * trace(5,1);

% Gauss-Seidel Communication
nrGaussSeidelIterations = pose_iterations + rotation_iterations;
informationExchangeGaussSeidel = pose_iterations.*(commLinks*sizePose) + rotation_iterations.*(commLinks*sizeRotation);


% Compare with DDF-SAM Communication
nrGNIterations = 1; % typically 3
sizePoseDDFSAM = 6*8;  % Assumed to be 12 doubles, 1 double = 8 byte
informationExchangeDDF = nrGNIterations.*((commLinks*sizePoseDDFSAM) + (commLinks*sizePoseDDFSAM).^2);
