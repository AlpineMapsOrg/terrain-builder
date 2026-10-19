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
    explicit LogScope(spdlog::level::level_enum new_level, spdlog::logger &logger = *Log::get_logger(), bool allow_loosen = false) {
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
