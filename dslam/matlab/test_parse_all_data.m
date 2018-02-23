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

% Set dataset_path and sequence_id.
parseAllData(dataset_path, sequence_id);

root = [dataset_path '/' sequence_id '/'];
dpath = [root 'dslam/'];
load([dpath 'full_data.mat']);

[~, matching] = pdist2(...
    vo_data.netvlad(1:128, :)', vo_data.netvlad(1:128, :)', ...
    'squaredeuclidean', 'Smallest', 10);
figure(1);
plot(kron(1:vo_data.n_frames, ones(1, 10)), matching(:), '.');
