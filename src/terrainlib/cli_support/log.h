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

#ifndef SPDLOG_FMT_EXTERNAL
#define SPDLOG_FMT_EXTERNAL
#define FMT_HEADER_ONLY
#endif
#include "fmt_impls.h"
#include "numeric/int_math.h"
#include <atomic>
#include <filesystem>
#include <map>
#include <string>

#include <fmt/core.h>
#include <spdlog/spdlog.h>

namespace cli_support {

class Log {
public:
	static void init(spdlog::level::level_enum level);
    // Also writes the log to path, appending, and flushes it from level info on.
    static void add_file(const std::filesystem::path& path);
    // Creates the logger on first use, at level info; safe to call from any thread.
    static const std::shared_ptr<spdlog::logger>& get_logger();
};

// Names accepted by the --verbosity option of the tools.
const std::map<std::string, spdlog::level::level_enum>& log_level_names();

// Assertion failures are bugs; logs them with their stack trace, so that they also
// reach a log file, before aborting.
void log_assertion_failures();

} // namespace cli_support

#define LOG_TRACE(...) SPDLOG_LOGGER_TRACE(::cli_support::Log::get_logger(), __VA_ARGS__)
#define LOG_DEBUG(...) SPDLOG_LOGGER_DEBUG(::cli_support::Log::get_logger(), __VA_ARGS__)
#define LOG_INFO(...)  SPDLOG_LOGGER_INFO (::cli_support::Log::get_logger(), __VA_ARGS__)
#define LOG_WARN(...)  SPDLOG_LOGGER_WARN (::cli_support::Log::get_logger(), __VA_ARGS__)
#define LOG_ERROR(...) SPDLOG_LOGGER_ERROR(::cli_support::Log::get_logger(), __VA_ARGS__)
#define LOG_ERROR_AND_EXIT(...) \
    do {                        \
        LOG_ERROR(__VA_ARGS__); \
        exit(1);                \
    } while (false)

// Logs at most once per call site for the lifetime of the process, even across threads.
#define LOG_WARN_ONCE(...)                                       \
    do {                                                         \
        static std::atomic<bool> _log_warn_once_fired { false }; \
        if (!_log_warn_once_fired.exchange(true)) {              \
            LOG_WARN(__VA_ARGS__);                               \
        }                                                        \
    } while (false)

namespace cli_support {

// Tracks call count per call site and reports whether the current call falls
// on a power-of-two occurrence (1, 2, 4, 8, 16, ...).
class LogBackoff {
public:
    bool should_log() {
        this->_count++;
        return is_power_of_two(this->_count);
    }

    uint64_t count() const {
        return this->_count;
    }

private:
    uint64_t _count = 0;
};

} // namespace cli_support

// Logs on occurrence 1, 2, 4, 8, 16, ... per call site, with the occurrence count appended.
#define LOG_WARN_BACKOFF(fmt_str, ...)                                  \
    do {                                                                \
        static ::cli_support::LogBackoff _log_warn_backoff;                            \
        if (_log_warn_backoff.should_log()) {                           \
            LOG_WARN(fmt_str " (occurrence {})", ##__VA_ARGS__, _log_warn_backoff.count()); \
        }                                                                \
    } while (false)
