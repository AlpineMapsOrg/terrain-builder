#pragma once

#include <algorithm>
#include <cmath>

#include "Range.h"
#include "geometry/bounds.h"

#include "bvh/NodeHeader.h"
#include "view/Camera.h"

namespace view {

// Which clusters show.
struct CutRule {
    // The largest error that may show, as a fraction of the screen height.
    double threshold;
    // Whether to leave out clusters out of view.
    bool cull = true;

    bool operator==(const CutRule &) const = default;
};

namespace detail {

// Projects `error` seen from `distance` by `camera` onto its screen, as a fraction of the screen height.
inline double project_error(const Camera &camera, const double error, const double distance) {
    // Nothing shows from closer than the near plane.
    const double shown_distance = std::max(distance, camera.z_near());
    return error / shown_distance / (2.0 * std::tan(camera.fov_y() / 2.0));
}

}

// Checks whether the cut for `camera` under `rule` can pass between the errors of `header`, seen from within `distances`.
// It can when the smallest error fits at the farthest distance and the largest parent error does not fit at the nearest.
[[nodiscard]]
inline bool crosses_cut(const bvh::NodeHeader &header, const RangeInclusive<double> &distances, const Camera &camera, const CutRule &rule) {
    const bool error_fits = detail::project_error(camera, header.min_child_error, distances.end) <= rule.threshold;
    const bool parent_error_fits = detail::project_error(camera, header.max_parent_error, distances.start) <= rule.threshold;
    return error_fits && !parent_error_fits;
}

// Checks whether `error` fits the cut, that is, is small enough to draw, measured from the nearest point of `bounds`.
[[nodiscard]]
inline bool is_small_enough_to_draw(const double error, const bvh::Bounds &bounds, const Camera &camera, const CutRule &rule) {
    const double nearest = geometry::compute_nearest_distance_to_bounds(bounds, camera.eye());
    return detail::project_error(camera, error, nearest) <= rule.threshold;
}

// Checks whether the cut can cross the leaf from anywhere within its LOD bounds.
[[nodiscard]]
inline bool should_visit_leaf(const bvh::NodeHeader &header, const Camera &camera, const CutRule &rule) {
    const RangeInclusive<double> distances = geometry::compute_distance_range_to_bounds(header.lod_bounds, camera.eye());
    return crosses_cut(header, distances, camera, rule);
}

}
