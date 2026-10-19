#pragma once

#include <optional>
#include <string>

#include <fmt/format.h>

#include "error/Error.h"
#include "gdal/CplError.h"

namespace srs {

// No transformation exists from one spatial reference to another.
struct MissingTransform {
    std::string source;
    std::string target;
    // The error GDAL recorded for the failure, if it recorded one.
    std::optional<gdal::CplError> gdal_error;

    bool operator==(const MissingTransform &) const = default;
};

inline std::string format_as(const MissingTransform &error) {
    if (error.gdal_error.has_value()) {
        return fmt::format("no transformation exists from {} to {}: {}", error.source, error.target, error.gdal_error->message());
    }
    return fmt::format("no transformation exists from {} to {}", error.source, error.target);
}

// A transformation from one spatial reference to another cannot map at least one point.
struct Untransformable {
    std::string source;
    std::string target;
    // The error GDAL recorded for the failure, if it recorded one. (skipped after 20 errors)
    std::optional<gdal::CplError> gdal_error;

    bool operator==(const Untransformable &) const = default;
};

inline std::string format_as(const Untransformable &error) {
    if (error.gdal_error.has_value()) {
        return fmt::format("a point cannot be transformed from {} to {}: {}", error.source, error.target, error.gdal_error->message());
    }
    return fmt::format("a point cannot be transformed from {} to {}", error.source, error.target);
}

// An error from transforming between two spatial references.
using TransformError = alp::Error<MissingTransform, Untransformable>;

}
