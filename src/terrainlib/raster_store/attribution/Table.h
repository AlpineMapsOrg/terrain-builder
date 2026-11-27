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
#include <limits>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "error/MessageCode.h"
#include "error/Result.h"
#include "error/utils.h"

namespace raster_store::attribution {

inline constexpr uint32_t index_limit = std::numeric_limits<uint16_t>::max();

struct Entity {
    double spatial_resolution;
    std::string acquisition_date;
    std::string ingestion_date;
    std::string copyright;
    std::string copyright_link;
    std::string license;

    bool operator==(const Entity&) const = default;
};

struct EmptyTable : alp::MessageCode<"the attribution table cannot be empty"> {};

// The table has more entities than there are attribution indices.
struct TooManyEntities {
    size_t count;

    bool operator==(const TooManyEntities&) const = default;
};

inline std::string format_as(const TooManyEntities& error) {
    return fmt::format("the table has {} entities, but attribution indices only reach {}", error.count, index_limit);
}

using MakeTableError = alp::Error<EmptyTable, TooManyEntities>;
// The sources that attribution indices refer to: entity i describes the source with index i. It holds at least one and at most index_limit entities.
class Table {
public:
    // The table with `entities`, if it has at least one and at most index_limit.
    [[nodiscard]]
    static alp::Result<Table, MakeTableError> from_entities(std::vector<Entity> entities);

    // The entities, in index order.
    std::span<const Entity> entities() const {
        return this->_entities;
    }

private:
    explicit Table(std::vector<Entity> entities)
        : _entities(std::move(entities)) {}

    std::vector<Entity> _entities;
};

inline alp::Result<Table, MakeTableError> Table::from_entities(std::vector<Entity> entities) {
    if (entities.empty()) {
        return alp::make_error(EmptyTable{});
    }
    if (entities.size() > index_limit) {
        return alp::make_error(TooManyEntities{entities.size()});
    }
    return Table(std::move(entities));
}

} // namespace raster_store::attribution
