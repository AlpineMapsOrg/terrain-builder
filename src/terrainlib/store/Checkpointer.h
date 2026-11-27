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

#include <atomic>
#include <chrono>
#include <memory>

#include "error/Result.h"
#include "error/log.h"
#include "store/Index.h"
#include "store/IndexFormat.h"
#include "store/Draft.h"

namespace store {

inline constexpr std::chrono::steady_clock::duration default_checkpoint_interval = std::chrono::minutes(2);

// Saves a draft's index once an interval has passed since the last save.
// Copies share the time of the last save.
class Checkpointer {
public:
    explicit Checkpointer(const std::chrono::steady_clock::duration interval = default_checkpoint_interval)
        : _interval(interval),
          _last_save(std::make_shared<std::atomic<std::chrono::steady_clock::time_point>>(std::chrono::steady_clock::now())) {}

    // Saves the index of `draft` if the interval has passed since the last save, and otherwise does nothing.
    // May be called concurrently; of the calls that find the interval passed, only one saves.
    template <Index IndexType, typename Payload>
    [[nodiscard]]
    alp::Result<void, WriteIndexError> save_if_due(Draft<IndexType, Payload>& draft) {
        const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point last_save = this->_last_save->load();
        if (now - last_save < this->_interval) {
            return {};
        }

        // Another call that found the interval passed has already claimed this save.
        if (!this->_last_save->compare_exchange_strong(last_save, now)) {
            return {};
        }

        return draft.save_index();
    }

    // Saves the index of `draft` if the interval has passed since the last save, and logs a warning if saving fails.
    template <Index IndexType, typename Payload>
    void save_if_due_or_warn(Draft<IndexType, Payload>& draft) {
        // A failed checkpoint only costs progress after a crash: publishing saves the index again and fails if it cannot.
        LOG_WARN_IF_ERROR(this->save_if_due(draft), "checkpointing the draft");
    }

private:
    std::chrono::steady_clock::duration _interval;
    std::shared_ptr<std::atomic<std::chrono::steady_clock::time_point>> _last_save;
};

} // namespace store
