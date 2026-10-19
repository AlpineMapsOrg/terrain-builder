#pragma once

#include <cstdint>
#include <source_location>
#include <span>
#include <string>

#include <cpptrace/utils.hpp>
#include <fmt/format.h>
#include <libassert/assert.hpp>

#include "enumerate.h"
#include "error/Error.h"
#include "error/ErrorCode.h"
#include "error/Trace.h"
#include "span_utils.h"

template <alp::ErrorCode... Es>
struct fmt::formatter<alp::BasicError<Es...>> {
    constexpr fmt::format_parse_context::iterator parse(fmt::format_parse_context &context) {
        auto position = context.begin();
        if (position != context.end() && *position == '#') {
            this->_form = Form::Chain;
            ++position;
        } else if (position != context.end() && *position == '?') {
            this->_form = Form::Report;
            ++position;
        }
        if (position != context.end() && *position != '}') {
            throw fmt::format_error("an error formats with {}, {:#} or {:?}");
        }
        return position;
    }

    fmt::format_context::iterator format(const alp::BasicError<Es...> &error, fmt::format_context &context) const {
        const std::span<const alp::Frame> frames = error.frames();
        const std::string cause = error._code.message();
        fmt::format_context::iterator output = context.out();

        const auto [origin, outer_frames] = *ASSERT_VAL(split_first(frames));

        switch (this->_form) {
        case Form::Message:
            return fmt::format_to(output, "{}", cause);

        case Form::Chain:
            output = fmt::format_to(output, "{}", cause);
            for (const alp::Frame &frame : frames) {
                if (frame.message.has_value()) {
                    output = fmt::format_to(output, ": {}", *frame.message);
                }
            }
            return output;

        case Form::Report:
            if (origin->message.has_value()) {
                output = fmt::format_to(output, "{}\n    ", *origin->message);
            }
            output = fmt::format_to(output, "{}\n    {}", format_location(origin->location), cause);
            if (outer_frames.empty()) {
                return output;
            }
            output = fmt::format_to(output, "\n\nTrace:");
            for (const auto [number, frame] : enumerate<uint32_t>(outer_frames)) {
                output = fmt::format_to(output, "\n    {}: ", number);
                if (frame.message.has_value()) {
                    output = fmt::format_to(output, "{}\n       ", *frame.message);
                }
                output = fmt::format_to(output, "{}", format_location(frame.location));
            }
            return output;
        }
        return output;
    }

private:
    // How an error was asked to format.
    enum class Form {
        // The cause message
        Message,

        // Its cause and all of its messages on one line, innermost first.
        Chain,

        // Every place it was, innermost first, each with what was being done there and the cause at the origin.
        Report,
    };

    static std::string format_location(const std::source_location location) {
        return fmt::format("{} at {}:{}:{}", cpptrace::prune_symbol(location.function_name()), location.file_name(), location.line(), location.column());
    }

    Form _form = Form::Message;
};
