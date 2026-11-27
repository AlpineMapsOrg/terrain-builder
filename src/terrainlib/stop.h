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

#include <stop_token>

#include "error/MessageCode.h"
#include "error/Result.h"
#include "error/utils.h"

// The work stopped because a stop was requested.
struct Stopped : alp::MessageCode<"the work was stopped on request"> {};

// Checks that no stop was requested through `token`.
[[nodiscard]]
inline alp::Result<void, Stopped> check_not_stopped(const std::stop_token& token) {
    if (token.stop_requested()) {
        return alp::make_error(Stopped{});
    } else {
        return {};
    }
}
