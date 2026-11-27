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

#include "raster_store/attribution/table_file.h"

#include <array>
#include <filesystem>
#include <limits>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

#include <cpl_json.h>
#include <fmt/format.h>
#include <fmt/std.h>

#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "gis/CplError.h"
#include "io/bytes.h"
#include "io/filesystem.h"
#include "path_utils.h"

namespace raster_store::attribution {

namespace {

// The most bytes GDAL's JSON parser accepts, since it takes the length as an int.
constexpr size_t max_json_bytes = std::numeric_limits<int>::max();

} // namespace

alp::Result<Table, ParseError> parse(const std::string_view json) {
    // Parse the JSON
    if (json.size() > max_json_bytes) {
        return alp::make_error(Unsupported{}, fmt::format("the table has {} bytes, more than the {} the JSON parser accepts", json.size(), max_json_bytes));
    }
    CPLJSONDocument document;
    auto [loaded, gdal_error] = gis::capture_error([&] {
        return document.LoadMemory(reinterpret_cast<const GByte*>(json.data()), static_cast<int>(json.size()));
    });
    if (!loaded) {
        return alp::make_error(CorruptTable{}, "the table is not valid JSON", std::move(gdal_error));
    }

    // Check the array
    const CPLJSONObject root = document.GetRoot();
    if (root.GetType() != CPLJSONObject::Type::Array) {
        return alp::make_error(CorruptTable{}, "the table is not a JSON array");
    }
    const CPLJSONArray array = root.ToArray();

    // Read the slots
    std::vector<Entity> entities;
    entities.reserve(static_cast<size_t>(array.Size()));
    for (int index = 0; index < array.Size(); index++) {
        const CPLJSONObject object = array[index];
        if (object.GetType() != CPLJSONObject::Type::Object) {
            return alp::make_error(CorruptTable{}, fmt::format("slot {} is not an object", index));
        }
        const CPLJSONObject::Type resolution_type = object["spatial_resolution"].GetType();
        if (resolution_type != CPLJSONObject::Type::Double && resolution_type != CPLJSONObject::Type::Integer && resolution_type != CPLJSONObject::Type::Long) {
            return alp::make_error(CorruptTable{}, fmt::format("the spatial_resolution of slot {} is not a number", index));
        }
        constexpr std::array fields{"acquisition_date", "ingestion_date", "copyright", "copyright_link", "license"};
        for (const auto field : fields) {
            if (object[field].GetType() != CPLJSONObject::Type::String) {
                return alp::make_error(CorruptTable{}, fmt::format("the {} of slot {} is not a string", field, index));
            }
        }
        entities.push_back({object.GetDouble("spatial_resolution"), object.GetString("acquisition_date"), object.GetString("ingestion_date"), object.GetString("copyright"), object.GetString("copyright_link"), object.GetString("license")});
    }

    ALP_TRY(Table table, Table::from_entities(std::move(entities)));
    return table;
}

alp::Result<std::filesystem::path, FindTableError> find_table(const std::filesystem::path& storage_path) {
    ALP_TRY(const std::filesystem::path absolute_path, ::io::filesystem::absolute(storage_path));
    std::filesystem::path directory = paths::normalized(absolute_path);
    for (int level = 0; level < 3; level++) {
        const std::filesystem::path path = directory / file_name;
        // A broken symlink counts as the table, so that a storage never falls back to an ancestor's table without notice.
        ALP_TRY(const bool is_symlink, ::io::filesystem::is_symlink(path));
        if (is_symlink) {
            return path;
        }
        ALP_TRY(const bool exists, ::io::filesystem::exists(path));
        if (exists) {
            return path;
        }
        directory = directory.parent_path();
    }
    return alp::make_error(MissingTable{storage_path});
}

alp::Result<Table, ReadTableError> read_table(const std::filesystem::path& storage_path) {
    ALP_TRY(const std::filesystem::path path, find_table(storage_path));

    const auto context = alp::make_context("reading the attribution table at {}", path);
    ALP_TRY_CONTEXT(const ::io::bytes::vector bytes, ::io::bytes::read(path), context);
    const std::string_view json(reinterpret_cast<const char*>(bytes.data()), bytes.size());
    ALP_TRY_CONTEXT(Table table, parse(json), context);

    return table;
}

alp::Result<void, CopyTableError> copy_table(const std::filesystem::path& source_storage_path, const std::filesystem::path& destination_directory) {
    ALP_TRY(const std::filesystem::path source, find_table(source_storage_path));
    const std::filesystem::path destination = destination_directory / file_name;

    ALP_TRY(::io::filesystem::copy_file(source, destination));

    return {};
}

} // namespace raster_store::attribution
