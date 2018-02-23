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

function plotDataOverTime(data_increments, data_increment_times, vo_end_time)

opt_traffic = ...
    cumsum(cellfun(@(x) sum(sum(x(:, :, 1))), data_increments));
netvlad_traffic = ...
    cumsum(cellfun(@(x) sum(sum(x(:, :, 2))), data_increments));
gv_traffic = ...
    cumsum(cellfun(@(x) sum(sum(x(:, :, 3))), data_increments));

opt_filter = [true; opt_traffic(2:end) ~= opt_traffic(1:end-1)];

nv_filt = [true; netvlad_traffic(2:end) ~= netvlad_traffic(1:end-1)];
gv_filt = [true; gv_traffic(2:end) ~= gv_traffic(1:end-1)];

plot(data_increment_times(opt_filter), opt_traffic(opt_filter) / 1e6, '-x');
hold on;
plot(data_increment_times(nv_filt), netvlad_traffic(nv_filt) / 1e6);
plot(data_increment_times(gv_filt), gv_traffic(gv_filt) / 1e6);
y_lim = ylim;
plot([vo_end_time vo_end_time], ylim, 'k--');
set(gca, 'YLim', y_lim);
hold off;
legend('DOpt', 'DVPR', 'RelPose', 'VO end time', 'Location', 'NorthWest');
title('Total data transmission');
xlabel('time [s]');
ylabel('total transmitted [MB]');
grid on;

end

