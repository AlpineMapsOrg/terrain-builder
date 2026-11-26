/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
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
#include <queue>
#include <type_traits>

#include <libassert/assert.hpp>

#include "error/ResultFn.h"
#include "error/unwrap.h"
#include "hierarchy/IndexMap.h"
#include "hierarchy/Traits.h"
#include "store/KeyTraits.h"

namespace hierarchy {

enum class TraversalOrder {
    DepthFirst,
    BreadthFirst,
};

struct AlwaysRefine {
    template <typename Key>
    constexpr bool operator()(const Key&) const {
        return true;
    }
};

namespace detail {

// What `Visit` returns for a node of `Keys`.
template <typename Visit, typename Keys>
using VisitResult = std::invoke_result_t<Visit &, const typename Keys::Key &, NodeStatus>;

// Visits `current` and then, depth first, the nodes below each node that `refine_fn` accepts.
template <store::KeyTraits Keys, Traits Tree, typename Visit, typename RefineFn>
VisitResult<Visit, Keys> walk_depth_first(const IndexMap<Keys, Tree> &index, Visit &visit, RefineFn &refine_fn, const typename Keys::Key &current) {
    const std::optional<NodeStatus> status_opt = index.get(current);
    if (!status_opt.has_value()) {
        return {};
    }

    auto visit_result = visit(current, status_opt.value());
    if (!visit_result.has_value() || !refine_fn(current)) {
        return visit_result;
    }

    const auto children_opt = Tree::children(current);
    if (!children_opt.has_value()) {
        return visit_result;
    }
    for (const auto &child : children_opt.value()) {
        auto child_result = walk_depth_first(index, visit, refine_fn, child);
        if (!child_result.has_value()) {
            return child_result;
        }
    }
    return visit_result;
}

// Visits the nodes level by level from `root`, below each node that `refine_fn` accepts.
template <store::KeyTraits Keys, Traits Tree, typename Visit, typename RefineFn>
VisitResult<Visit, Keys> walk_breadth_first(const IndexMap<Keys, Tree> &index, Visit &visit, RefineFn &refine_fn, const typename Keys::Key &root) {
    std::queue<typename Keys::Key> queue;
    queue.push(root);

    while (!queue.empty()) {
        const typename Keys::Key current = queue.front();
        queue.pop();

        const std::optional<NodeStatus> status_opt = index.get(current);
        if (!status_opt.has_value()) {
            continue;
        }

        auto visit_result = visit(current, status_opt.value());
        if (!visit_result.has_value()) {
            return visit_result;
        }

        if (refine_fn(current)) {
            const auto children_opt = Tree::children(current);
            if (children_opt.has_value()) {
                for (const auto &child : children_opt.value()) {
                    queue.push(child);
                }
            }
        }
    }
    return {};
}

// Visits the nodes from `root` in `order`, and stops at the first failure of `visit`.
template <store::KeyTraits Keys, Traits Tree, typename Visit, typename RefineFn>
auto walk(const IndexMap<Keys, Tree> &index, Visit &visit, RefineFn &refine_fn, const typename Keys::Key &root, const TraversalOrder order) {
    switch (order) {
    case TraversalOrder::DepthFirst:
        return walk_depth_first(index, visit, refine_fn, root);
    case TraversalOrder::BreadthFirst:
        return walk_breadth_first(index, visit, refine_fn, root);
    }
    UNREACHABLE("invalid traversal order", order);
}

} // namespace detail

// Visits the nodes of `index` from `root` in `order` with their status, and descends into the children of each node that `refine_fn` accepts.
// If `visit_fn` returns a result, the first failure ends the walk and is returned.
template <store::KeyTraits Keys, Traits Tree, typename VisitFn, typename RefineFn = AlwaysRefine>
[[nodiscard]]
auto traverse(
    const IndexMap<Keys, Tree> &index,
    VisitFn &&visit_fn,
    RefineFn &&refine_fn = {},
    const typename Keys::Key &root = Tree::root(),
    TraversalOrder order = TraversalOrder::DepthFirst) {
    // A visitor that returns nothing is called as one whose result never holds a failure, so that one walk serves both.
    auto visit = alp::make_result_fn(visit_fn);
    return alp::unwrap_infallible(detail::walk(index, visit, refine_fn, root, order));
}

} // namespace hierarchy
