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

#include <cstdint>

#include "io/bytes.h"

namespace io::hash {

enum class Algorithm : uint8_t {
    // No hash; its hash has no bytes.
    None,
    // XXH3-64 with seed 0; its hash is the 8 bytes in big-endian order, the canonical xxHash form.
    Xxh3_64,
};

// Hashes `bytes` with `algorithm`.
bytes::vector data(bytes::span bytes, Algorithm algorithm);

} // namespace io::hash
