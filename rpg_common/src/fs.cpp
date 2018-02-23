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

#include "rpg_common/fs.h"

#include <dirent.h>
#include <errno.h>
#include <sys/stat.h>

#include <glog/logging.h>

namespace rpg_common
{
namespace fs
{

bool fileExists(const std::string& path)
{
  struct stat st;
  return stat(path.c_str(), &st) == 0 && (st.st_mode & S_IFREG);
}

bool pathExists(const std::string& path)
{
  struct stat st;
  return stat(path.c_str(), &st) == 0 && (st.st_mode & S_IFDIR);
}

void splitPathAndFilename(
    const std::string& str, std::string* path, std::string* filename)
{
  CHECK_NOTNULL(path)->clear();
  CHECK_NOTNULL(filename)->clear();
  const size_t right_delim = str.find_last_of("/");
  if (right_delim != std::string::npos)
  {
    *path = str.substr(0, right_delim);
  }
  *filename = str.substr(right_delim + 1);
}

// Returns full paths. No recursion.
void getFilesAndSubfolders(const std::string& path,
                           std::vector<std::string>* files,
                           std::vector<std::string>* folders)
{
  CHECK_NOTNULL(files)->clear();
  CHECK_NOTNULL(folders)->clear();
  CHECK(pathExists(path));

  DIR* directory_stream = CHECK_NOTNULL(opendir(path.c_str()));
  struct dirent* directory_entry;
  while ((directory_entry = readdir(directory_stream)) != NULL)
  {
    const std::string filename(directory_entry->d_name);
    if ((filename == ".") || (filename == "..")) {
      continue;
    }
    const std::string abs_path = path + "/" + filename;

    if (fileExists(abs_path))
    {
      files->emplace_back(abs_path);
    }
    else
    {
      CHECK(pathExists(abs_path));
      folders->emplace_back(abs_path);
    }
  }

  closedir(directory_stream);
}

}  // namespace fs
}  // namespace rpg_common
