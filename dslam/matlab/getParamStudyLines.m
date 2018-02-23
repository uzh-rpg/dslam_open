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

function lines = getParamStudyLines(param, values, runs, dpath, params)
% Returns lines with data: 
% opt, dvpr, relpose(word_assoc), relpose(desc_assoc), ate

n = numel(values);

for line_i = 1:5
    for x_i = 1:n
        lines(line_i).result{x_i} = [];
        lines(line_i).avg(x_i) = 0;
        lines(line_i).failures(x_i) = 0;
    end
end

for x_i = 1:n
    for run_i = runs
        % Tardioli enabled
        params.use_tardioli = 1;
        params.(param) = values(x_i);
        params.run_i = run_i;
        load([dpath 'run_data_' num2str(cell2mat(struct2cell(params))') ...
            '.mat']);
        
        % opt, dvpr, relpose(word_assoc), ate
        for line_i = 1:3
            run_result = ...
                sum(cellfun(@(x) sum(sum(x(:, :, line_i))), ...
                data_increments));
            lines(line_i).result{x_i} = [lines(line_i).result{x_i} ...
                run_result];
        end
        if (numel(accuracy_measurements{end}.optimized_groups) > 1)
            lines(5).failures(x_i) = lines(5).failures(x_i) + 1;
        else
            ate = accuracy_measurements{end}.optimized_groups(1).ATE;
            lines(5).result{x_i} = [lines(5).result{x_i} ate];
        end
        
        % relpose(desc_assoc)
        params.use_tardioli = 0;
        load([dpath 'run_data_' num2str(cell2mat(struct2cell(params))') ...
            '.mat']);
        run_result = ...
            sum(cellfun(@(x) sum(sum(x(:, :, 3))), data_increments));
        lines(4).result{x_i} = [lines(4).result{x_i} ...
            run_result];
    end
end
for line_i = 1:5
    for x_i = 1:n
        if numel(lines(line_i).result{x_i}) > 0
            lines(line_i).avg(x_i) = mean(lines(line_i).result{x_i});
        else
            lines(line_i).avg(x_i) = 0;
        end
    end
end

end

