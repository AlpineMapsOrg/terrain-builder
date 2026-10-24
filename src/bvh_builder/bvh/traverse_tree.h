#pragma once

#include <cstdint>
#include <ranges>
#include <type_traits>
#include <vector>

#include <libassert/assert.hpp>

#include "error/Result.h"
#include "error/ResultFn.h"
#include "error/try.h"
#include "io/Error.h"

#include "bvh/NodeCache.h"
#include "bvh/Batch.h"
#include "bvh/Node.h"
#include "bvh/NodeView.h"
#include "bvh/Reference.h"

namespace bvh {

// What a traversal does after visiting a node.
class Control {
public:
    static const Control Continue;
    static const Control Prune;
    static const Control Break;

    // Continues on true and breaks off on false.
    constexpr Control(const bool keep_going) : _action(Action::Break) {
        if (keep_going) {
            this->_action = Action::Continue;
        }
    }

    bool operator==(const Control &) const = default;

private:
    enum class Action {
        Continue,
        Prune,
        Break
    };

    constexpr explicit Control(const Action action) : _action(action) {}

    Action _action;
};

inline constexpr Control Control::Continue{Action::Continue};
inline constexpr Control Control::Prune{Action::Prune};
inline constexpr Control Control::Break{Action::Break};

// Visits the tree from `root` in pre-order, passing each node with the reference naming it and skipping the children of every node `v` prunes.
// False if `v` broke off the walk before its end.
// Fails with `E` if a batch cannot be read or `v` fails.
template <typename E = io::Error, typename F>
[[nodiscard]]
alp::Result<bool, E> traverse_tree(NodeCache &nodes, const Reference &root, F &&v) {
    auto visit = alp::make_result_fn<E>(v);

    std::vector<Reference> stack{root};

    while (!stack.empty()) {
        const Reference reference = stack.back();
        stack.pop_back();

        ALP_TRY(const std::shared_ptr<const Batch> shared_batch, nodes.read_batch(reference.batch));
        const Batch &batch = *shared_batch;
        const NodeView view = batch.resolve(batch.node(reference.node));

        // A visitor that returns nothing always continues.
        Control control = Control::Continue;
        if constexpr (std::is_void_v<typename decltype(visit(reference, view))::value_type>) {
            ALP_TRY(visit(reference, view));
        } else {
            ALP_TRY(control, visit(reference, view));
        }

        if (control == Control::Break) {
            return false;
        }

        // Pushed in reverse so that they come off the stack in stored order.
        if (view.is_inner() && control == Control::Continue) {
            stack.append_range(view.children() | std::views::reverse);
        }
    }

    // We walked to the end
    return true;
}

}
