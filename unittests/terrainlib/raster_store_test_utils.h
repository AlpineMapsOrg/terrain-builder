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

#include <algorithm>
#include <filesystem>
#include <string>
#include <string_view>

#include <catch2/catch_test_macros.hpp>
#include <radix/raster.h>

#include "../result_helpers.h"
#include "io/WriteMode.h"
#include "io/text.h"
#include "raster_store/attribution/table_file.h"
#include "raster_store/storage.h"

namespace raster_store_test {

inline constexpr std::string_view entity_json =
    R"({"spatial_resolution":0.25,"acquisition_date":"summer 2020","ingestion_date":"2026-09-08",)"
    R"("copyright":"Example","copyright_link":"https://example.org","license":"Example license"})";

inline void write_text(const std::filesystem::path& path, const std::string_view text)
{
    REQUIRE_VALUE(io::text::write(text, path, { .mode = io::WriteMode::Overwrite }));
}

inline void write_table(const std::filesystem::path& directory)
{
    write_text(directory / raster_store::attribution::file_name, "[" + std::string(entity_json) + "]");
}

inline raster_store::storage::CreateOptions small_options()
{
    raster_store::storage::CreateOptions options;
    options.nominal_tile_size = 4;
    return options;
}

template <typename T>
void check_bytes(const radix::Raster<T>& left, const radix::Raster<T>& right)
{
    CHECK(left.size() == right.size());
    CHECK(std::ranges::equal(left.bytes(), right.bytes()));
}

} // namespace raster_store_test
