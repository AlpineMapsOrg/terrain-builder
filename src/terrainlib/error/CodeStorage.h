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

#include <type_traits>

#include "error/ErasedCode.h"
#include "error/TypedCode.h"

namespace alp {

// How an error with the codes Es stores its code: a TypedCode, or an ErasedCode when Es is empty.
template <typename... Es>
using CodeStorage = std::conditional_t<sizeof...(Es) == 0, ErasedCode, TypedCode<Es...>>;

} // namespace alp
