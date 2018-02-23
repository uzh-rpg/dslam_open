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

#pragma once

#include <iostream>
#include <map>
#include <set>
#include <utility>
#include <vector>

// With this, you can simply pass stl containers to ostreams.
// This not only prints its contents, but also handles indentation.
// For currently supported stl containers, see std::ostream << specializations
// below.

namespace std {

template <typename Type>
ostream& rpgCommonStreamUtilOut (
    ostream& stream, const size_t indent_level, const Type& item)
{
  return stream << item;
}

inline std::ostream& rpgCommonStreamUtilIndent(
    std::ostream& stream, const size_t indent_level)
{
  if (indent_level == 0u)
  {
    return stream;
  }
  else
  {
    return rpgCommonStreamUtilIndent(stream << "  ", indent_level - 1u);
  }
}

template <typename Type1, typename Type2>
ostream& rpgCommonStreamUtilOut (
    ostream& stream, const size_t indent_level,
    const map<Type1, Type2>& map)
{
  stream << "{" << endl;
  for(const typename std::map<Type1, Type2>::value_type& pair : map)
  {
    rpgCommonStreamUtilOut(rpgCommonStreamUtilOut(
        rpgCommonStreamUtilIndent(stream, indent_level + 1u),
        indent_level + 1u, pair.first) << ": ", indent_level + 1u, pair.second)
            << endl;
  }
  return rpgCommonStreamUtilIndent(stream, indent_level) << "}";
}

template <typename Type>
ostream& rpgCommonStreamUtilOut (
    ostream& stream, const size_t indent_level, const set<Type>& set)
{
  stream << "{" << endl;
  for(const Type& item : set)
  {
    rpgCommonStreamUtilOut(rpgCommonStreamUtilIndent(
        stream, indent_level + 1u), indent_level + 1u, item) <<
            endl;
  }
  return rpgCommonStreamUtilIndent(stream, indent_level) << "}";
}

template <typename Type>
ostream& rpgCommonStreamUtilOut (ostream& stream, const size_t indent_level,
                                 const vector<Type>& vector)
{
  stream << "{" << endl;
  for(const Type& item : vector)
  {
    rpgCommonStreamUtilOut(rpgCommonStreamUtilIndent(
        stream, indent_level + 1u), indent_level + 1u, item) << endl;
  }
  return rpgCommonStreamUtilIndent(stream, indent_level) << "}";
}

template <typename Type1, typename Type2>
ostream& operator<< (ostream& stream, const pair<Type1, Type2>& pair)
{
  return stream << "(" << pair.first << ", " << pair.second << ")";
}

template <typename Type1, typename Type2>
ostream& operator<< (ostream& stream, const map<Type1, Type2>& map)
{
  return rpgCommonStreamUtilOut(stream, 0u, map);
}

template <typename Type>
ostream& operator<< (ostream& stream, const set<Type>& set)
{
  return rpgCommonStreamUtilOut(stream, 0u, set);
}

template <typename Type>
ostream& operator<< (ostream& stream, const vector<Type>& vector)
{
  return rpgCommonStreamUtilOut(stream, 0u, vector);
}

}  // namespace std
