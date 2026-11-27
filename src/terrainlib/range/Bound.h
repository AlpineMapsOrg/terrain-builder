/*****************************************************************************
 * AlpineMaps.org
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

#include <optional>
#include <utility>

#include "numeric/checked_cast.h"

enum class BoundKind {
    Included,
    Excluded,
    Unbounded,
};

template <typename T>
struct Bound {
    BoundKind kind;
    std::optional<T> value;

    constexpr Bound(const BoundKind kind_value, std::optional<T> value_value) : kind(kind_value), value(std::move(value_value)) {}

    template <typename U>
    explicit constexpr Bound(const Bound<U> &other)
        : Bound(other.kind, other.value.transform([](const U value) { return debug_checked_cast<T>(value); })) {}

    [[nodiscard]] static constexpr Bound included(T value) noexcept {
        return {BoundKind::Included, std::move(value)};
    }

    [[nodiscard]] static constexpr Bound excluded(T value) noexcept {
        return {BoundKind::Excluded, std::move(value)};
    }

    [[nodiscard]] static constexpr Bound unbounded() noexcept {
        return {BoundKind::Unbounded, std::nullopt};
    }

    [[nodiscard]] constexpr bool operator==(const Bound &) const noexcept = default;
};
