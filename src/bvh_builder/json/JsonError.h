#pragma once

#include <string>
#include <utility>

#include <fmt/format.h>
#include <glaze/core/context.hpp>

namespace json {

// A value that could not be written as JSON, or text that could not be read as one.
class JsonError {
public:
    explicit JsonError(const glz::error_code code, std::string location) : _code(code), _location(std::move(location)) {}

    bool operator==(const JsonError &) const = default;

    // What went wrong.
    glz::error_code code() const {
        return this->_code;
    }

    // The line and column in the text where reading stopped, with the surrounding text.
    const std::string &location() const {
        return this->_location;
    }

private:
    glz::error_code _code;
    std::string _location;
};

}

template <>
struct fmt::formatter<json::JsonError> {
    constexpr fmt::format_parse_context::iterator parse(fmt::format_parse_context &context) {
        return context.begin();
    }

    fmt::format_context::iterator format(const json::JsonError &error, fmt::format_context &context) const {
        return fmt::format_to(context.out(), "the JSON is invalid at {}", error.location());
    }
};
