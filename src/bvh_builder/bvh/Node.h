#pragma once

#include <utility>

#include "io/glm_serialization.h"
#include "io/radix_serialization.h"
#include "octree/id_serialization.h"

#include "bvh/NodeHeader.h"
#include "bvh/Reference.h"

namespace bvh {

class Batch;

// A node of the bvh, whose contents its batch resolves.
class Node {
public:
    using Header = NodeHeader;

    // zpp bits wants a default ctor
    Node() = default;

    bool operator==(const Node &) const = default;

    const NodeHeader &header() const {
        return this->_header;
    }

    bool is_leaf() const noexcept {
        return this->_count == 0;
    }
    bool is_inner() const noexcept {
        return !this->is_leaf();
    }

private:
    friend Batch;
    friend zpp::bits::access;
    using serialize = zpp::bits::members<3>;

    Node(NodeHeader header, const NodeIndex start, const NodeIndex count)
        : _header(std::move(header)), _start(start), _count(count) {}

    NodeIndex start() const {
        return this->_start;
    }

    NodeIndex count() const {
        return this->_count;
    }

    NodeHeader _header;
    NodeIndex _start; // the first child, or the source of a leaf
    NodeIndex _count; // how many children it has, 0 for a leaf
};

}
