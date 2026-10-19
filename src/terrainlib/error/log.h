#pragma once

#include <fmt/format.h>

#include "error/format.h"
#include "log.h"
#include "macros.h"

// Logs `error` as a report of its causes and its trace, with the message formatted from `...`, if any, as its outermost context.
#define LOG_UNEXPECTED(error, ...) LOG_ERROR("{:?}", auto(error) __VA_OPT__(.context(::fmt::format(__VA_ARGS__))))

// LOG_UNEXPECTED_OF(result, ...) with its result variable spelled out.
#define ALP_LOG_UNEXPECTED_OF_IMPL(logged, result, ...)                    \
    do {                                                                   \
        const auto &logged = (result);                                     \
        if (!logged.has_value()) {                                         \
            LOG_UNEXPECTED(logged.error() __VA_OPT__(, ) __VA_ARGS__);     \
        }                                                                  \
    } while (false)

// Evaluates `result` and, if it failed, logs its error like LOG_UNEXPECTED.
#define LOG_UNEXPECTED_OF(result, ...) ALP_LOG_UNEXPECTED_OF_IMPL(ALP_UNIQUE_IDENT(_alp_logged_result_), result __VA_OPT__(, ) __VA_ARGS__)
