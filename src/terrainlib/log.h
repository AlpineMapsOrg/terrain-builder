#ifndef LOG_H
#define LOG_H

#ifndef SPDLOG_FMT_EXTERNAL
#define SPDLOG_FMT_EXTERNAL
#define FMT_HEADER_ONLY
#endif
#include <fmt/core.h>
#include <spdlog/spdlog.h>

class Log {
private:
	static std::shared_ptr<spdlog::logger> logger;
public:
	static void init(spdlog::level::level_enum logLevel);

	inline static std::shared_ptr<spdlog::logger>& get_logger() {
        if (Log::logger == nullptr) {
            Log::init(spdlog::level::level_enum::trace);
        }
		return Log::logger;
	}
};

#define LOG_TRACE(...) ::Log::get_logger()->trace(__VA_ARGS__)
#define LOG_DEBUG(...) ::Log::get_logger()->debug(__VA_ARGS__)
#define LOG_INFO(...)  ::Log::get_logger()->info(__VA_ARGS__)
#define LOG_WARN(...)  ::Log::get_logger()->warn(__VA_ARGS__)
#define LOG_ERROR(...) ::Log::get_logger()->error(__VA_ARGS__)
#define LOG_FATAL(...) ::Log::get_logger()->critical(__VA_ARGS__)

#endif
