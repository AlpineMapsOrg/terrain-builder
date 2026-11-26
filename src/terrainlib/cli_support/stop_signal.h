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

#include <memory>
#include <stop_token>

namespace cli_support::stop_signal {

struct Installation;
// While a guard lives, SIGINT and SIGTERM request a stop through the tokens `token` returns.
// The handlers in place before the first guard come back once the last one is destroyed.
using Guard = std::shared_ptr<const Installation>;

// Makes SIGINT and SIGTERM request a stop through the tokens `token` returns, until the guard is destroyed.
[[nodiscard]] Guard install();

// A token that asks to stop once SIGINT or SIGTERM arrived while a guard lived.
[[nodiscard]] std::stop_token token();

bool requested();

// 128 + the received signal, or 1 if no signal was received.
int exit_code();

} // namespace cli_support::stop_signal
