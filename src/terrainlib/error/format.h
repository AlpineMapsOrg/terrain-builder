#pragma once

#include <cstdint>
#include <span>
#include <string>

#include <fmt/format.h>

#include "enumerate.h"
#include "error/Error.h"
#include "error/ErrorCode.h"
#include "error/Trace.h"

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
            output = fmt::format_to(output, "{}", cause);
            output = format_frames(output, frames);
            for (const alp::ErasedError *source = error.source(); source != nullptr; source = source->source()) {
                output = fmt::format_to(output, "\n\nCaused by: {}", *source);
                output = format_frames(output, source->frames());
            }
            return output;
        }
        return output;
    }

private:
    // Writes each of `frames` on a line of its own, numbered from the origin.
    static fmt::format_context::iterator format_frames(fmt::format_context::iterator output, const std::span<const alp::Frame> frames) {
        for (const auto [number, frame] : enumerate<uint32_t>(frames)) {
            output = fmt::format_to(output, "\n    {}: {}", number, frame);
        }
        return output;
    }

    // How an error was asked to format.
    enum class Form {
        // The cause message.
        Message,

        // Its cause and all of its messages on one line, innermost first.
        Chain,

        // Its cause and every place it was, one per line starting at its origin, then the same for each error it was created from, the one it was directly created from first.
        Report,
    };

    Form _form = Form::Message;
};
