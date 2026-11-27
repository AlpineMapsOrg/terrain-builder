#pragma once

#include <span>
#include <utility>
#include <variant>

#include <libassert/assert.hpp>

#include "variant_utils.h"

#include "bvh/NodeHeader.h"
#include "bvh/Reference.h"
#include "dag/SourceId.h"

namespace bvh {

class NodeView {
public:
    using Children = std::span<const Reference>;
    using Contents = std::variant<Children, dag::SourceId>;

    NodeView(const NodeHeader &header, const Contents contents)
        : _header(header), _contents(contents) {}

    const NodeHeader &header() const {
        return this->_header;
    }

    bool is_leaf() const {
        return std::holds_alternative<dag::SourceId>(this->_contents);
    }

    bool is_inner() const {
        return !this->is_leaf();
    }

    Children children() const {
        DEBUG_ASSERT(this->is_inner());
        return std::get<Children>(this->_contents);
    }

    // The group or cluster the leaf stands for.
    const dag::SourceId &source() const {
        DEBUG_ASSERT(this->is_leaf());
        return std::get<dag::SourceId>(this->_contents);
    }

    // Calls whichever of `visitors` matches what the node holds.
    template <typename... F>
    decltype(auto) visit(F &&...visitors) const {
        return match(this->_contents, std::forward<F>(visitors)...);
    }

private:
    NodeHeader _header;
    Contents _contents;
};

}
