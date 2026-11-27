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

#include <string_view>

#include <catch2/catch_test_macros.hpp>
#include <radix/tile.h>

#include "raster_store/path_layout/zoom_x_y_google.h"

TEST_CASE("raster XYZ paths round trip boundary IDs and reject malformed paths", "[raster_store][path_layout]")
{
    const auto layout = raster_store::path_layout::zoom_x_y_google::layout();
    CHECK(layout.id == "zoom/x/y_google");
    for (const radix::tile::Id key : { radix::tile::Id { 0, { 0, 0 } }, radix::tile::Id { 32, { UINT32_MAX, UINT32_MAX } } }) {
        CHECK(layout.identify_node(layout.locate_node(key)) == key);
    }
    for (const std::string_view path : { "", "/0/0/0", "0/1/0", "33/0/0", "1/0/0.amort", "1/0/0/extra", "1/0/0junk", "1/../0" }) {
        INFO(path);
        CHECK_FALSE(layout.identify_node(path).has_value());
    }
}
