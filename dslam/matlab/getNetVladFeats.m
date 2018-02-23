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

function getNetVladFeats(dataset_path, sequence_id, data_type)
% Parses and saves netvlad feats.
%
% Requires the netvlad matlab code as dependency, as well as
% vd16_pitts30k_conv5_3_vlad_preL2_intra_white.mat, create a symlink 
% folder "netvlad" to the folder containing it.

root = [dataset_path '/' sequence_id '/'];
if (strcmp(data_type, 'kitti'))
    impath = [root 'image_0/'];
else
    assert(strcmp(data_type, 'stata'))
    impath = [root 'images/left/'];
end
dpath = [root 'dslam/'];
if (exist(dpath, 'dir') ~= 7)
    mkdir(dpath);
end

feat_file = [dpath 'netvlad_feats.bin'];
%%
if ~exist(feat_file, 'file')
    net_file = 'netvlad/vd16_pitts30k_conv5_3_vlad_preL2_intra_white.mat';
    load(net_file, 'net');
    assert(exist('net', 'var') == 1);
    net= relja_simplenn_tidy(net);
    images = files_with_ext(impath, 'jpg');
    %%
    assert(numel(images) > 0);
    serialAllFeats(net, '/', ...
        images, feat_file, 'batchSize', 4);
end

end
