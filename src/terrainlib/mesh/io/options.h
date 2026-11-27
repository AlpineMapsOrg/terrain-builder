/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2025 Martin Braunsperger
 * Copyright (C) 2025 Adam Celarek-Litofcenko
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *****************************************************************************/

#pragma once

#include <string>

#include "io/WriteMode.h"
#include "io/image/Format.h"
#include "io/json/Value.h"

namespace mesh::io {

struct LoadOptions {
};

struct SaveOptions {
    ::io::image::Format texture_format = ::io::image::Format::Jpeg;
    std::string name = "mesh";
    ::io::json::Object metadata = {};
    ::io::WriteMode mode = ::io::default_write_mode;
    bool make_dirs = ::io::default_make_dirs;
};

} // namespace mesh::io
