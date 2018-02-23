// Copyright (C) 2017-2018 Titus Cieslewski, RPG, University of Zurich,
//   Switzerland
//   You can contact the author at <titus at ifi dot uzh dot ch>
// Copyright (C) 2017-2018 Davide Scaramuzza, RPG, University of Zurich, 
//   Switzerland
//
// This file is part of dslam_open.
//
// dslam_open is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// dslam_open is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with dslam_open. If not, see <http://www.gnu.org/licenses/>.

#include "rpg_common/cv_type.h"

#include <opencv2/core/core.hpp>

namespace rpg_common {

#define CV_TYPE(type, _value) \
template <> \
const int CvType<type>::value = _value;

CV_TYPE(unsigned char, CV_8U);
CV_TYPE(char, CV_8S);
CV_TYPE(uint16_t, CV_16U);
CV_TYPE(int16_t, CV_16S);
CV_TYPE(int32_t, CV_32S);
CV_TYPE(float, CV_32F);
CV_TYPE(double, CV_64F);

}  // namespace rpg_common
