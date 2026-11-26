/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Martin Braunsperger
 * Copyright (C) 2026 Adam Celarek-Litofcenko
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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <thread>
#include <utility>

#include <radix/tile.h>

#include "cli_support/ProgressIndicator.h"

namespace tile_downloader {

class TileLogger {
public:
    class Session;

    explicit TileLogger(unsigned root_zoom_level)
        : _root_zoom_level(root_zoom_level), _progress(PROGRESS_RESOLUTION) {}

    [[nodiscard]] Session start();

    // The tile turned out not to exist, so its branch stops here.
    void missing(const radix::tile::Id &tile) {
        this->close_branch(tile);
    }

    // We chose not to descend into this tile's branch (already on disk, or past --max-zoom-level).
    void skipped(const radix::tile::Id &tile) {
        this->close_branch(tile);
    }

private:
    static constexpr size_t PROGRESS_RESOLUTION = 10'000;

    unsigned _root_zoom_level;
    cli_support::ProgressIndicator _progress;
    std::jthread _progress_thread;
    size_t _steps_done = 0;
    std::map<unsigned, uint32_t> _level_counts;

    void start_monitoring() {
        this->_progress_thread = this->_progress.start_monitoring();
    }

    void finish_monitoring() {
        while (this->_steps_done < PROGRESS_RESOLUTION) {
            this->_progress.task_finished();
            this->_steps_done++;
        }
        if (this->_progress_thread.joinable()) {
            this->_progress_thread.join();
        }
    }

    void cancel_monitoring() noexcept {
        if (!this->_progress_thread.joinable()) {
            return;
        }

        this->_progress_thread.request_stop();
        try {
            this->_progress_thread.join();
        } catch (...) {
            // Session cleanup must not replace the active exception.
        }
    }

    // Every tile at `level` covers an equal 4^-(level - root) share of the whole
    // download tree, so summing that share for every closed branch gives the
    // fraction of the tree that is done.
    double completed_fraction() const {
        double fraction = 0.0;
        for (const auto &[level, count] : this->_level_counts) {
            fraction += static_cast<double>(count) / std::pow(4.0, static_cast<double>(level - this->_root_zoom_level));
        }
        return fraction;
    }

    void close_branch(const radix::tile::Id &tile) {
        this->_level_counts[tile.zoom_level]++;
        this->advance_progress();
    }

    void advance_progress() {
        const auto target = std::min(PROGRESS_RESOLUTION,
                                      static_cast<size_t>(this->completed_fraction() * double(PROGRESS_RESOLUTION)));
        while (this->_steps_done < target) {
            this->_progress.task_finished();
            this->_steps_done++;
        }
    }
};

class TileLogger::Session {
public:
    explicit Session(TileLogger &logger)
        : _logger(&logger) {
        this->_logger->start_monitoring();
    }

    Session(const Session &) = delete;
    Session &operator=(const Session &) = delete;

    Session(Session &&other) noexcept
        : _logger(std::exchange(other._logger, nullptr)) {}

    Session &operator=(Session &&) = delete;

    ~Session() {
        if (this->_logger) {
            this->_logger->cancel_monitoring();
        }
    }

    void finish() {
        this->_logger->finish_monitoring();
        this->_logger = nullptr;
    }

private:
    TileLogger *_logger;
};

inline TileLogger::Session TileLogger::start() {
    return Session(*this);
}

} // namespace tile_downloader
