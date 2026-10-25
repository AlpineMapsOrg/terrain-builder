#pragma once

#include "bvh/NodeHeader.h"
#include "view/Camera.h"
#include "view/CutRule.h"
#include "view/Frustum.h"

namespace view {

// The camera and rule a cut is selected for.
class CutView {
public:
    CutView(const Camera &camera, const CutRule &rule)
        : _camera(camera), _frustum(Frustum::from_camera(camera)), _rule(rule) {}

    // Checks whether `bounds` are in view, as they always are if the rule does not cull.
    [[nodiscard]]
    bool is_in_view(const bvh::Bounds &bounds) const {
        return !this->_rule.cull || intersects(bounds, this->_frustum);
    }

    // Checks whether `error` is small enough to draw, measured from the nearest point of `bounds`.
    [[nodiscard]]
    bool is_small_enough_to_draw(const double error, const bvh::Bounds &bounds) const {
        return view::is_small_enough_to_draw(error, bounds, this->_camera, this->_rule);
    }

    // Checks whether the subtree below `header` could contain any of the cut.
    [[nodiscard]]
    bool could_contain_cut(const bvh::NodeHeader &header) const {
        return this->is_in_view(header.tight_bounds) && !this->is_small_enough_to_draw(header.max_parent_error, header.lod_bounds);
    }

    // Checks whether the cut can cross the leaf below `header`.
    [[nodiscard]]
    bool could_cross_leaf(const bvh::NodeHeader &header) const {
        return this->is_in_view(header.tight_bounds) && should_visit_leaf(header, this->_camera, this->_rule);
    }

private:
    Camera _camera;
    Frustum _frustum;
    CutRule _rule;
};

}
