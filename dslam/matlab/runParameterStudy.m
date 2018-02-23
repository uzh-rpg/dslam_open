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

function runParameterStudy(params_to_vary, param_values, robot_vo_data, ...
    decentr_stream, dvpr_train_path, distributed_mapper_location, ...
    dpath, params, rerun)

assert(numel(params_to_vary) == numel(param_values));

parampowerset = {params};
for param_i = 1:numel(params_to_vary)
    augmented = {};
    varied_param = params_to_vary{param_i};
    for set_i = 1:numel(parampowerset)
        paramset = parampowerset{set_i};
        for param_val = param_values{param_i}
            eval(['paramset.' varied_param ' = ' num2str(param_val) ';']);
            augmented = [augmented; paramset];
        end
    end
    parampowerset = augmented;
end

disp(['Will run ' num2str(numel(parampowerset)) ' simulations']);
disp('Please confirm');
%pause();

for run_i = 1:numel(parampowerset)
    paramset = parampowerset{run_i};
    if (exist([dpath 'run_data_' ...
            num2str(cell2mat(struct2cell(paramset))') '.mat'], 'file') ...
            == 2)
        if (~rerun)
            warning('Given paramset run already exists, skipping!');
            continue;
        end
    end
    
    runSimulationAndSave(robot_vo_data, decentr_stream, ...
        dvpr_train_path, distributed_mapper_location, dpath, paramset);
end

end
