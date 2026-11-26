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

#include <spdlog/spdlog.h>

#include "containers/UniqueResource.h"
#include "log.h"

class LogScope {
private:
    struct Inner {
        spdlog::logger *logger;
        spdlog::level::level_enum original_level;
    };

    struct ResetLevel {
        void operator()(Inner &inner) const noexcept {
            inner.logger->set_level(inner.original_level);
        }
    };

public:
    explicit LogScope(spdlog::level::level_enum new_level, spdlog::logger &logger = *logging::Log::get_logger(), bool allow_loosen = false) {
        const auto original_level = logger.level();
        // Only set to if it's more restrictive than current level
        if (allow_loosen || new_level >= original_level) {
            logger.set_level(new_level);

            this->_inner.reset(Inner{&logger, original_level});
        }
    }

private:
    UniqueResource<Inner, ResetLevel> _inner;
};
