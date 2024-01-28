#include "log.h"

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

std::shared_ptr<spdlog::logger> Log::Logger;

void Log::init(spdlog::level::level_enum logLevel) {
	spdlog::set_pattern("[%Y-%m-%d %T.%e] [%=3n] [%^%l%$] %v");
	Logger = spdlog::stderr_color_mt("LOG");
	Logger->set_level(logLevel);
}
