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

#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "../result_helpers.h"
#include "raster_store/attribution/Table.h"

namespace {

namespace attribution = raster_store::attribution;

} // namespace

TEST_CASE("from_entities rejects an empty table and more than index_limit entities", "[raster_store][attribution][Table]")
{
    CHECK(REQUIRE_ERROR(attribution::Table::from_entities({})).holds<attribution::EmptyTable>());

    const auto error = REQUIRE_ERROR(attribution::Table::from_entities(std::vector<attribution::Entity>(attribution::index_limit + 1)));
    CHECK(error.holds_code(attribution::TooManyEntities { attribution::index_limit + 1 }));

    const attribution::Table table = REQUIRE_VALUE(attribution::Table::from_entities(std::vector<attribution::Entity>(attribution::index_limit)));
    CHECK(table.entities().size() == attribution::index_limit);
}
