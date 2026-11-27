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

#include <expected>

#include "error/Error.h"
#include "error/ErrorCode.h"

namespace alp {

// The underlying result type; a value, or an error of type E.
template <typename T, typename E>
using BasicResult = std::expected<T, E>;

// A value, or an error with codes of type Es; with no Es, an erased error.
template <typename T, typename... Es>
using Result = BasicResult<T, Error<Es...>>;

// A value, or an erased error.
template <typename T>
using ErasedResult = BasicResult<T, ErasedError>;

// The underlying unexpected type; an error of type E, to return from a function that returns a BasicResult.
template <typename E>
using BasicUnexpected = std::unexpected<E>;

// An error with codes of type Es, to return from a function that returns a Result.
template <typename... Es>
using Unexpected = BasicUnexpected<Error<Es...>>;

} // namespace alp
