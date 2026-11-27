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

#include <fmt/format.h>

#include "cli_support/log.h"
#include "error/format.h"
#include "macros.h"

// Implementation of LOG_REPORT and LOG_WARN_REPORT
#define LOG_REPORT_IMPL(log_macro, error, ...) log_macro("{:?}", auto(error) __VA_OPT__(.context(::fmt::format(__VA_ARGS__))))

// Logs `error` as a report of its causes and its trace, with the message formatted from `...`, if any, as its outermost context.
#define LOG_REPORT(error, ...) LOG_REPORT_IMPL(LOG_ERROR, error __VA_OPT__(, ) __VA_ARGS__)

// Logs `error` like LOG_REPORT, as a warning.
#define LOG_WARN_REPORT(error, ...) LOG_REPORT_IMPL(LOG_WARN, error __VA_OPT__(, ) __VA_ARGS__)

// LOG_IF_ERROR and LOG_WARN_IF_ERROR with the logging macro and the result variable spelled out.
#define LOG_IF_ERROR_IMPL(log_macro, logged, result, ...)              \
    do {                                                               \
        const auto &logged = (result);                                 \
        if (!logged.has_value()) {                                     \
            log_macro(logged.error() __VA_OPT__(, ) __VA_ARGS__);      \
        }                                                              \
    } while (false)

// Evaluates `result` and, if it failed, logs its error like LOG_REPORT.
#define LOG_IF_ERROR(result, ...) LOG_IF_ERROR_IMPL(LOG_REPORT, ALP_UNIQUE_IDENT(_alp_logged_result_), result __VA_OPT__(, ) __VA_ARGS__)

// Evaluates `result` and, if it failed, logs its error like LOG_WARN_REPORT.
#define LOG_WARN_IF_ERROR(result, ...) LOG_IF_ERROR_IMPL(LOG_WARN_REPORT, ALP_UNIQUE_IDENT(_alp_logged_result_), result __VA_OPT__(, ) __VA_ARGS__)
