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

#include <optional>
#include <source_location>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "PrunedLocation.h"

namespace alp {

// A point an error was at: where it originated, was propagated or was given context.
struct Frame {
    // What was being done at this point, absent where the error was only propagated.
    std::optional<std::string> message = std::nullopt;

    // Where in the source this point is.
    std::source_location location;
};

// The frame's message, if it has one, and where it is.
inline std::string format_as(const Frame &frame) {
    if (frame.message.has_value()) {
        return fmt::format("{} ({})", *frame.message, PrunedLocation{frame.location});
    } else {
        return fmt::to_string(PrunedLocation{frame.location});
    }
}

// The points an error was at, starting where it originated.
class Trace {
public:
    // Starts a trace for an error that originates at `origin`.
    explicit Trace(const std::source_location origin) {
        this->_frames.push_back(Frame{.location = origin});
    }

    // Starts a trace for an error that originates at `origin` with `message` as what was being done there.
    Trace(const std::source_location origin, std::string message) {
        this->_frames.push_back(Frame{.message = std::move(message), .location = origin});
    }

    void push(Frame frame) {
        this->_frames.push_back(std::move(frame));
    }

    // Where the error originated.
    [[nodiscard]]
    std::source_location origin() const {
        return this->_frames.front().location;
    }

    // The points the error was at, innermost first, starting at the origin.
    [[nodiscard]]
    std::span<const Frame> frames() const {
        return this->_frames;
    }

private:
    std::vector<Frame> _frames;
};

} // namespace alp
