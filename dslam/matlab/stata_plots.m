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

load(regexprep([dpath 'run_data_' ...
    num2str(cell2mat(struct2cell(params))') '.mat'], '\s+', ' '));

%% Ground truth
figure(1);
gt = load([root 'gt_poses.txt']);
gt = gt(:, 4:4:end);
plot(gt(:, 1) - gt(1, 1), gt(:, 2) - gt(1, 2));
axis equal;
title('Stata 2012-01-25-12-14')
xlabel('x[m]');
ylabel('y[m]');
grid on;

set(gcf, 'Position', [0 0 350 260]);
if ~exist('plots','dir'), mkdir('plots'); end
eval('export_fig plots/stata_gt.pdf -transparent -nocrop');

%%
figure(2);
plotDecentrState(vo_end_state, false);
title('Stata 2012-01-25-12-14')
xlabel('x[m]');
ylabel('y[m]');
grid on;

set(gcf, 'Position', [0 0 350 260]);
if ~exist('plots','dir'), mkdir('plots'); end
eval('export_fig plots/stata_vo_end.pdf -transparent -nocrop');

%%
figure(3);
plotDecentrState(decentr_state, false);
title('Stata 2012-01-25-12-14')
xlabel('x[m]');
ylabel('y[m]');
grid on;

set(gcf, 'Position', [0 0 350 260]);
if ~exist('plots','dir'), mkdir('plots'); end
eval('export_fig plots/stata_oops.pdf -transparent -nocrop');

%%
figure(2);
set(gcf, 'Position', [0 0 380 260]);
eval('export_fig plots/data-time-stata.pdf -transparent -nocrop');
