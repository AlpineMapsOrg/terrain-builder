/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2025 Martin Braunsperger
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

#include <filesystem>
#include <glm/glm.hpp>
#include <radix/geometry.h>
#include <radix/tile.h>

#include <fmt/format.h>
#include <fmt/std.h>

namespace fmt {
template <glm::length_t N, typename T>
struct formatter<glm::vec<N, T>> {
    template <typename ParseContext>
    constexpr auto parse(ParseContext &ctx) {
        return ctx.begin();
    }

    template <typename FormatContext>
    auto format(const glm::vec<N, T> &vec, FormatContext &ctx) const {
        auto out = fmt::format_to(ctx.out(), "(");
        for (glm::length_t i = 0; i < N; i++) {
            out = fmt::format_to(out, "{}{}", i == 0 ? "" : ", ", vec[i]);
        }
        return fmt::format_to(out, ")");
    }
};

template <glm::length_t N, typename T>
struct formatter<radix::geometry::Aabb<N, T>> {
    template <typename ParseContext>
    constexpr auto parse(ParseContext &ctx) {
        return ctx.begin();
    }

    template <typename FormatContext>
    auto format(const radix::geometry::Aabb<N, T> &aabb, FormatContext &ctx) const {
        return fmt::format_to(ctx.out(), "[{}-{}]", aabb.min, aabb.max);
    }
};

// Aabb2 derives from Aabb instead of aliasing it, so it needs its own specialization.
template <typename T>
struct formatter<radix::geometry::Aabb2<T>> : formatter<radix::geometry::Aabb<2, T>> {};

template <>
struct formatter<radix::tile::Id> : formatter<std::string_view> {
    template <typename FormatContext>
    auto format(const radix::tile::Id &id, FormatContext &ctx) const {
        return formatter<std::string_view>::format(radix::tile::to_string(id), ctx);
    }
};
}
