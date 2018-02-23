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

#include <functional>
#include <unordered_set>

#include <glog/logging.h>

namespace rpg_common {

template <typename ... CallbackArgs>
class CallbackHost
{
 public:
  typedef std::function<void(CallbackArgs...)> Callback;

  void addCallback(const Callback& callback)
  {
    CHECK(callback) << "Callback isn't a valid function!";
    CHECK(callbacks_.emplace(callback).second)
    << "Duplicate callback addition!";
  }

 protected:
  void triggerCallbacks(CallbackArgs... args) const
  {
    for (const Callback& callback : callbacks_)
    {
      callback(args...);
    }
    VLOG(40) << callbacks_.size() << " callbacks processed.";
  }
 private:
  std::unordered_set<Callback> callbacks_;
};

}  // namespace rpg_common

namespace std {

template <typename ... CallbackArgs>
struct hash<std::function<void(CallbackArgs...)>>
{
  size_t operator()(const std::function<void(CallbackArgs...)>& x) const
  {
    CHECK(x);
    void (*const* pointer)(CallbackArgs...) =
        x.template target<void(*)(CallbackArgs...)>();
    if (pointer)  // Functor is a function pointer.
    {
      return std::hash<const void*>()(reinterpret_cast<const void*>(pointer));
    }
    else  // Functor is a lambda.
    {
      return std::hash<std::string>()(x.target_type().name());
    }
  }
};

template <typename ... CallbackArgs>
bool operator ==(
    const std::function<void(CallbackArgs...)>& a,
    const std::function<void(CallbackArgs...)>& b)
{
  CHECK(a);
  CHECK(b);
  void (*const* a_pointer)(CallbackArgs...) =
      a.template target<void(*)(CallbackArgs...)>();
  void (*const* b_pointer)(CallbackArgs...) =
      b.template target<void(*)(CallbackArgs...)>();
  if (a_pointer)  // Functor is a function pointer.
  {
    return a_pointer == b_pointer;
  }
  else  // Functor is a lambda.
  {
    return a.target_type().name() == b.target_type().name();
  }
}

}  // namespace std
namespace rpg = rpg_common;
