#pragma once

#include <concepts>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#include <cpl_error.h>
#include <libassert/assert.hpp>

namespace gdal {

// A failed GDAL call, with the category and message GDAL recorded for it.
class CplError {
public:
    explicit CplError(const CPLErrorNum code, std::string message) : _code(code), _message(std::move(message)) {
        DEBUG_ASSERT(code != CPLE_None);
    }

    // The error GDAL last recorded on this thread, if it recorded one.
    static std::optional<CplError> try_get_last_error() {
        const CPLErrorNum code = CPLGetLastErrorNo();
        if (code == CPLE_None) {
            return std::nullopt;
        }
        return CplError(code, CPLGetLastErrorMsg());
    }

    // The error GDAL last recorded on this thread, or an application-defined one if it recorded none.
    static CplError get_last_error() {
        return try_get_last_error().value_or(CplError(CPLE_AppDefined, "GDAL recorded no error"));
    }

    // Whether both errors have the same category.
    bool operator==(const CplError &other) const {
        return this->_code == other._code;
    }

    // The category GDAL assigned.
    CPLErrorNum code() const {
        return this->_code;
    }

    // The message GDAL recorded.
    const std::string &message() const {
        return this->_message;
    }

private:
    CPLErrorNum _code;
    std::string _message;
};

inline std::string_view format_as(const CplError &error) {
    return error.message();
}

// Runs `call` with a cleared GDAL error state and returns its result with the error GDAL recorded during it, if it recorded one.
template <std::invocable Call>
std::pair<std::invoke_result_t<Call>, std::optional<CplError>> capture_error(Call call) {
    CPLErrorReset();
    auto value = call();
    return {std::move(value), CplError::try_get_last_error()};
}

} // namespace gdal
