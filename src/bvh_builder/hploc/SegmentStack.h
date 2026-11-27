#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include <libassert/assert.hpp>

namespace hploc {

// A stack of segments, each a list of items with the depth at which it splits from the segment below.
template <typename T>
class SegmentStack {
public:
    // A view of a segment, valid until the stack next changes.
    struct SegmentView {
        // The depth at which it splits from the segment below.
        uint32_t depth;
        std::span<T> items;
    };

    // Pushes a segment holding only `item`.
    void push(const uint32_t depth, const T &item) {
        this->_segments.push_back(Segment{.depth = depth, .first_item = this->_items.size()});
        this->_items.push_back(item);
    }

    // The segment on top. The stack must not be empty.
    [[nodiscard]]
    SegmentView top() {
        DEBUG_ASSERT(!this->_segments.empty());
        const Segment &segment = this->_segments.back();
        return SegmentView{.depth = segment.depth, .items = std::span(this->_items).subspan(segment.first_item)};
    }

    // Joins the segment on top into the one below it, which then holds the items of both and keeps its own depth.
    // The stack must hold at least two segments.
    void join_top() {
        DEBUG_ASSERT(this->_segments.size() >= 2);
        // Their items already lie next to each other at the end of the list.
        this->_segments.pop_back();
    }

    // Keeps only the first `count` items of the segment on top, which must hold that many.
    void shrink_top(const size_t count) {
        DEBUG_ASSERT(count <= this->top().items.size());
        this->_items.resize(this->_segments.back().first_item + count);
    }

    // How many segments are on the stack.
    [[nodiscard]]
    size_t size() const {
        return this->_segments.size();
    }

private:
    struct Segment {
        uint32_t depth;
        // Where its items start in the list.
        size_t first_item;
    };

    std::vector<Segment> _segments;
    // The items of all segments, one segment after another.
    std::vector<T> _items;
};

}
