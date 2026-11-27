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
#include <type_traits>

#include "type_utils.h"

namespace alp {

// A callable that takes `Args` and returns a result holding a `T`.
template <typename F, typename T, typename... Args>
concept FallibleFn = std::invocable<F, Args...>
    && is_specialization_of_v<std::remove_cvref_t<std::invoke_result_t<F, Args...>>, std::expected>
    && std::same_as<typename std::remove_cvref_t<std::invoke_result_t<F, Args...>>::value_type, T>;

} // namespace alp
