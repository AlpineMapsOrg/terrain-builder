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

#include "io/hash.h"

#define XXH_INLINE_ALL
#include <xxhash.h>

#include <libassert/assert.hpp>

#include "range_utils.h"

namespace io::hash {
namespace {

    bytes::vector xxh3_64(const bytes::span bytes) {
        XXH64_canonical_t canonical;
        XXH64_canonicalFromHash(&canonical, XXH3_64bits(bytes.data(), bytes.size()));
        return to_vector<uint8_t>(canonical.digest);
    }

} // namespace

bytes::vector data(const bytes::span bytes, const Algorithm algorithm) {
    switch (algorithm) {
    case Algorithm::None:
        return {};
    case Algorithm::Xxh3_64:
        return xxh3_64(bytes);
    }
    PANIC("unsupported hash algorithm", algorithm);
}

} // namespace io::hash
