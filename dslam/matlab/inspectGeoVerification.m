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

function inspectGeoVerification(...
    dpath, num_robots, robot_a, robot_b, decentr_state)

netvlad_match_stats = load(...
    [dpath 'netvlad_match_stats_' num2str(num_robots) '_robots.txt']);

%% Find relevant matches.
filter_ab = (netvlad_match_stats(:, 1) == robot_a) & ...
    (netvlad_match_stats(:, 3) == robot_b);
filter_ba = (netvlad_match_stats(:, 1) == robot_b) & ...
    (netvlad_match_stats(:, 3) == robot_a);
filter = filter_ab | filter_ba;
relevant_matches = netvlad_match_stats(filter, :)

%% Build batch geo verification request
verification_request = [];
for i = 1:size(relevant_matches, 1)
    verification_request = [verification_request; ...
        makeVerificationRequest(decentr_state, ...
        relevant_matches(i, 3), relevant_matches(i, 4), ...
        relevant_matches(i, 1), relevant_matches(i, 2))];
end
dlmwrite('inspect_request.txt', verification_request, ' ');

end
