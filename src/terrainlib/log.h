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
#include <optional>

#include <fmt/core.h>
#include <spdlog/spdlog.h>

namespace logging {

class Log {
public:
	static void init(spdlog::level::level_enum level);
    // Also writes the log to path, appending, and flushes it from level info on.
    static void add_file(const std::filesystem::path& path);
    // Creates the logger on first use, at level info; safe to call from any thread.
    static const std::shared_ptr<spdlog::logger>& get_logger();
};

} // namespace logging

#define LOG_TRACE(...) SPDLOG_LOGGER_TRACE(::logging::Log::get_logger(), __VA_ARGS__)
#define LOG_DEBUG(...) SPDLOG_LOGGER_DEBUG(::logging::Log::get_logger(), __VA_ARGS__)
#define LOG_INFO(...)  SPDLOG_LOGGER_INFO (::logging::Log::get_logger(), __VA_ARGS__)
#define LOG_WARN(...)  SPDLOG_LOGGER_WARN (::logging::Log::get_logger(), __VA_ARGS__)
#define LOG_ERROR(...) SPDLOG_LOGGER_ERROR(::logging::Log::get_logger(), __VA_ARGS__)
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

namespace logging {

// Counts the occurrences at a call site; the power-of-two occurrences (1, 2, 4,
// 8, 16, ...) are the ones to log.
class LogBackoff {
public:
    // Counts an occurrence and returns its number if it is to be logged; safe to
    // call from any thread.
    std::optional<uint64_t> count_occurrence()
    {
        const uint64_t occurrence = m_count.fetch_add(1, std::memory_order_relaxed) + 1;
        if (!is_power_of_two(occurrence)) {
            return std::nullopt;
        }
        return occurrence;
    }

private:
    std::atomic<uint64_t> m_count { 0 };
};

} // namespace logging

// Logs on occurrence 1, 2, 4, 8, 16, ... per call site, with the occurrence count appended.
#define LOG_WARN_BACKOFF(fmt_str, ...)                                                  \
    do {                                                                                \
        static ::logging::LogBackoff _log_warn_backoff;                             \
        if (const auto _log_warn_occurrence = _log_warn_backoff.count_occurrence()) {   \
            LOG_WARN(fmt_str " (occurrence {})", ##__VA_ARGS__, *_log_warn_occurrence); \
        }                                                                               \
    } while (false)
