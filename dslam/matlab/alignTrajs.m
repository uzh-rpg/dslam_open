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

function [optim_T_gt_O, ate] = alignTrajs(p_gt_C, p_O_C, initial_T_gt_O)

% state: T_gt_O_ate in tdrpyxyz form
state = [0 0 0 initial_T_gt_O(1:3, 4)']';

    function T_gt_O = apply(estim_state)
        dT = rpyxyzToT([atan(estim_state(1:3)) / 4; estim_state(4:6)]);
        T_gt_O = dT * initial_T_gt_O;
    end

    function error = alignError(estim_state)
        T_gt_O = apply(estim_state);
        p_gt_C_estim = T_gt_O(1:3, 1:3) * p_O_C + T_gt_O(1:3, 4);
        error = p_gt_C' - p_gt_C_estim';
        error = reshape(error, [], 1);
    end

optim_state = lsqnonlin(@alignError, state);

errs = reshape(alignError(optim_state), 3, []);
ate = mean(sqrt(sum(errs.^2, 1)));
optim_T_gt_O = apply(optim_state);

end

