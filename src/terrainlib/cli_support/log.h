/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2024 Martin Braunsperger
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

#include <map>
#include <string>

#include <spdlog/spdlog.h>

#include "../log.h"

namespace cli_support {

// Names accepted by the --verbosity option of the tools.
const std::map<std::string, spdlog::level::level_enum>& log_level_names();

// Assertion failures are bugs; logs them with their stack trace, so that they also
// reach a log file, before aborting.
void log_assertion_failures();

} // namespace cli_support
