/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
 * Copyright (C) 2026 Martin Braunsperger
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

#include <memory>
#include <string_view>

#include "common_error_codes.h"
#include "error/Result.h"
#include "mesh/SimpleMesh.h"
#include "store/Codec.h"

namespace mesh::codec {

// The codec that `id` names.
[[nodiscard]]
alp::Result<std::unique_ptr<store::Codec<mesh::Simple>>, Unsupported> resolve(std::string_view id);

} // namespace mesh::codec
