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

#include <concepts>
#include <expected>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <source_location>

#include <fmt/format.h>

#include "type_utils.h"

namespace alp {

template <typename... Es>
class BasicError;

// An enum or a class other than a string, an optional, an expected or an error, formattable with fmt; the formatted text is the code's message.
template <typename E>
concept ErrorCode = std::same_as<E, std::remove_cvref_t<E>>
    && (std::is_enum_v<E> || std::is_class_v<E>)
    && !is_specialization_of_v<E, std::basic_string>
    && !is_specialization_of_v<E, std::basic_string_view>
    && !is_specialization_of_v<E, std::optional>
    && !is_specialization_of_v<E, std::expected>
    && !std::same_as<E, std::source_location>
    && !is_specialization_of_v<E, BasicError>
    && fmt::is_formattable<E>::value;

// A code, or a set of codes that is not erased.
template <typename T>
concept ErrorCodes = ErrorCode<T> || (is_specialization_of_v<T, BasicError> && !T::is_erased);

} // namespace alp
