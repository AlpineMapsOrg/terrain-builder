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

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>

#include "containers/FixedString.h"

namespace io::envelope {

template <uint32_t Number, typename VersionedPayloadType>
struct Version {
    static constexpr uint32_t number = Number;
    using Payload = VersionedPayloadType;
};

namespace detail {

template <uint32_t Number, typename... Versions>
struct FindVersion;

template <uint32_t Number, typename First, typename... Rest>
struct FindVersion<Number, First, Rest...> {
    using type = std::conditional_t<First::number == Number, First, typename FindVersion<Number, Rest...>::type>;
};

template <uint32_t Number>
struct FindVersion<Number> {
    using type = void;
};

template <typename... Versions>
consteval bool versions_are_strictly_increasing() {
    constexpr uint32_t numbers[] = {Versions::number...};
    for (size_t index = 1; index < sizeof...(Versions); ++index) {
        if (numbers[index - 1] >= numbers[index]) {
            return false;
        }
    }
    return true;
}

template <typename From, typename To>
concept ConvertsFromPrevious = requires(From previous) {
    { To::from_previous(std::move(previous)) } -> std::same_as<To>;
};

template <typename VersionTuple, size_t... Indices>
consteval bool conversions_are_valid(std::index_sequence<Indices...>) {
    return (ConvertsFromPrevious<
                typename std::tuple_element_t<Indices, VersionTuple>::Payload,
                typename std::tuple_element_t<Indices + 1, VersionTuple>::Payload>
            && ...);
}

} // namespace detail

template <FixedString ClassName, typename... Versions>
struct PayloadSchema {
    static_assert(sizeof...(Versions) > 0, "a payload schema requires at least one version");
    static_assert(detail::versions_are_strictly_increasing<Versions...>(), "payload versions must be strictly increasing");

    using VersionTuple = std::tuple<Versions...>;
    static constexpr size_t version_count = sizeof...(Versions);

    template <size_t Index>
    using VersionAt = std::tuple_element_t<Index, VersionTuple>;

    using LatestVersion = VersionAt<version_count - 1>;
    using Latest = typename LatestVersion::Payload;

    static constexpr std::string_view class_name{ClassName.value, sizeof(ClassName.value) - 1};
    static constexpr uint32_t latest_version = LatestVersion::number;

    template <uint32_t Number>
    static constexpr bool supports_version = !std::is_void_v<typename detail::FindVersion<Number, Versions...>::type>;

    static constexpr bool supports_version_number(const uint32_t number) {
        return ((number == Versions::number) || ...);
    }

    template <uint32_t Number>
    using Payload = typename detail::FindVersion<Number, Versions...>::type::Payload;

    static_assert(version_count == 1
                      || detail::conversions_are_valid<VersionTuple>(
                          std::make_index_sequence<version_count - 1>{}),
                  "each payload version must provide from_previous for the preceding version");
};

} // namespace io::envelope
