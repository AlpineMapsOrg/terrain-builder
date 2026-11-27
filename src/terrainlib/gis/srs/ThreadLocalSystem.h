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

#include <memory>
#include <mutex>
#include <optional>

#include <tbb/enumerable_thread_specific.h>

#include "gis/srs/System.h"

namespace gis::srs {

// A spatial reference that several threads use at the same time, each through its own copy.
class ThreadLocalSystem {
public:
    explicit ThreadLocalSystem(const System &system) : _shared(system.clone()), _mutex(std::make_unique<std::mutex>()) {}

    // The calling thread's copy, made on its first call.
    [[nodiscard]]
    const System &get() const {
        std::optional<System> &copy = this->_copies.local();
        if (!copy) {
            // GDAL does not allow using one spatial reference from several threads at once, so the shared system is only read under the lock.
            const std::lock_guard lock(*this->_mutex);
            copy = this->_shared.clone();
        }
        return *copy;
    }

private:
    System _shared;
    std::unique_ptr<std::mutex> _mutex;
    mutable tbb::enumerable_thread_specific<std::optional<System>> _copies;
};

} // namespace gis::srs
