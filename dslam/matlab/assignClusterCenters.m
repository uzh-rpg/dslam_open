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

function decentr_state = assignClusterCenters(...
    decentr_state, dvpr_train_path, params)

relevant_params.netvlad_dim = params.netvlad_dim;
relevant_params.num_robots = params.num_robots;
relevant_params.clusters_per_robot = params.clusters_per_robot;

%% Load feats
train_feat_files = filesWithExt(dvpr_train_path, 'bin');
train_feats = cellfun(@(x) ...
    fread( fopen(x, 'rb'), inf, 'float32=>single'), train_feat_files, ...
    'UniformOutput', false);
train_feats = cellfun(@(x) ...
    reshape(x, 4096, []), train_feats, ...
    'UniformOutput', false);
train_feats = cell2mat(train_feats');
train_feats = train_feats(1:relevant_params.netvlad_dim, :);
%% Cluster centers
clust_dir = [dvpr_train_path '/dslam/'];
clust_out_file = [clust_dir 'clusters' ...
    num2str(cell2mat(struct2cell(relevant_params))') '.mat'];
if (~exist(clust_out_file, 'file'))
    [~, cluster_centers] = kmeans(...
        train_feats', relevant_params.num_robots * ...
        relevant_params.clusters_per_robot, ...
        'MaxIter', 1e6);
    if ~exist(clust_dir,'dir'), mkdir(clust_dir); end
    save(clust_out_file, 'cluster_centers');
else
    load(clust_out_file);
end

%% Assign
assert(relevant_params.num_robots == numel(decentr_state));
for i = 1:numel(decentr_state)
    % TODO extend to multiple per
    decentr_state{i}.cluster_center = cluster_centers(i, :)';
end

end

