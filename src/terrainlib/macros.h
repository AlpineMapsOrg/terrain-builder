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

#if defined(__has_builtin)
#define ALP_HAS_BUILTIN(x) __has_builtin(x)
#else
#define ALP_HAS_BUILTIN(x) 0
#endif

#define ALP_CAT_IDENT_IMPL(a, b) a##b
#define ALP_CAT_IDENT(a, b) ALP_CAT_IDENT_IMPL(a, b)
#define ALP_UNIQUE_IDENT(prefix) ALP_CAT_IDENT(prefix, __COUNTER__)

#define ALP_COUNT_ARGS_IMPL(_1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, N, ...) N

// The number of arguments, up to 16.
#define ALP_COUNT_ARGS(...) ALP_COUNT_ARGS_IMPL(__VA_ARGS__, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1)

// `x` forwarded as the value category it was declared with, like std::forward<decltype(x)>(x).
#define ALP_FORWARD(x) static_cast<decltype(x) &&>(x)
