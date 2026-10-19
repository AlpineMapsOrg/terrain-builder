#pragma once

#include <string_view>

namespace mesh {

// A mesh operation whose result CGAL cannot represent as a manifold mesh.
struct NonManifoldResult {
    bool operator==(const NonManifoldResult &) const = default;
};

inline std::string_view format_as(const NonManifoldResult) {
    return "the result is not manifold";
}

} // namespace mesh
