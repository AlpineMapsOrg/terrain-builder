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

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <cstdlib>

#include <libassert/assert.hpp>

#include "cli_support/log.h"

namespace cli_support {

const std::shared_ptr<spdlog::logger>& Log::get_logger()
{
    static const std::shared_ptr<spdlog::logger> logger = [] {
        spdlog::set_pattern("[%Y-%m-%d %T.%e] [%=3n] [%^%l%$] %v");
        auto created = spdlog::stderr_color_mt("LOG");
        created->set_level(spdlog::level::info);
        return created;
    }();
    return logger;
}

void Log::init(spdlog::level::level_enum log_level) { get_logger()->set_level(log_level); }

void Log::add_file(const std::filesystem::path& path)
{
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(path.string(), false);
    file_sink->set_pattern("[%Y-%m-%d %T.%e] [%n] [%l] %v");
    get_logger()->sinks().push_back(std::move(file_sink));
    get_logger()->flush_on(spdlog::level::info);
}

const std::map<std::string, spdlog::level::level_enum>& log_level_names()
{
    static const std::map<std::string, spdlog::level::level_enum> names {
        { "off", spdlog::level::off },
        { "critical", spdlog::level::critical },
        { "error", spdlog::level::err },
        { "warn", spdlog::level::warn },
        { "info", spdlog::level::info },
        { "debug", spdlog::level::debug },
        { "trace", spdlog::level::trace },
    };
    return names;
}

void log_assertion_failures()
{
    libassert::set_failure_handler([](const libassert::assertion_info& info) {
        LOG_ERROR("{}", info.to_string(0, libassert::color_scheme::blank));
        Log::get_logger()->flush();
        std::abort();
    });
}

} // namespace cli_support
