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

function parseAllData(dataset_path, sequence_id, data_type)

root = [dataset_path '/' sequence_id '/'];
dpath = [root 'dslam/'];

out_file = [dpath 'full_data.mat'];
%%
if (~exist(out_file, 'file'))
    %%
    vo_data = getVOData(dataset_path, sequence_id, data_type);
    
    %%
    feat_file = [dpath 'netvlad_feats.bin'];
    netvlad_time_file = [root 'times.txt'];
    assert(exist(feat_file, 'file') == 2);
    assert(exist(netvlad_time_file, 'file') == 2);
    
    netvlad_dim = 4096;
    netvlad_feats = reshape(fread(...
        fopen(feat_file, 'rb'), inf, 'float32=>single'), ...
        netvlad_dim, []);
    if (strcmp(data_type, 'kitti'))
        netvlad_times = load(netvlad_time_file);
    else
        assert(strcmp(data_type, 'stata'));
        netvlad_times = load(netvlad_time_file) / 1e9;
    end
    
    [~, netvlad_index] = pdist2(netvlad_times, vo_data.times, 'cityblock', ...
        'Smallest', 1);
    vo_data.netvlad = netvlad_feats(:, netvlad_index);
    
    save(out_file, 'vo_data', '-v7.3'); 
end

end

