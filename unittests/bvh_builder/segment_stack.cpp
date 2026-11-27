#include "../catch2_helpers.h"

#include <vector>

#include "range_utils.h"

#include "hploc/SegmentStack.h"

TEST_CASE("join_top joins the top two segments", "[hploc][SegmentStack]") {
    hploc::SegmentStack<char> stack;
    stack.push(0, 'a');
    stack.push(5, 'b');
    stack.push(3, 'c');

    // The joined segment keeps the depth of the lower one.
    stack.join_top();
    CHECK(stack.size() == 2);
    CHECK(stack.top().depth == 5);
    CHECK(to_vector(stack.top().items) == std::vector{'b', 'c'});

    stack.join_top();
    CHECK(stack.size() == 1);
    CHECK(stack.top().depth == 0);
    CHECK(to_vector(stack.top().items) == std::vector{'a', 'b', 'c'});
}

TEST_CASE("shrink_top keeps the first items", "[hploc][SegmentStack]") {
    hploc::SegmentStack<char> stack;
    stack.push(0, 'a');
    stack.push(0, 'b');
    stack.push(0, 'c');
    stack.join_top();

    stack.shrink_top(2);
    CHECK(to_vector(stack.top().items) == std::vector{'b', 'c'});

    stack.shrink_top(1);
    CHECK(to_vector(stack.top().items) == std::vector{'b'});
}

TEST_CASE("push starts after the kept items", "[hploc][SegmentStack]") {
    hploc::SegmentStack<char> stack;
    stack.push(0, 'a');
    stack.push(0, 'b');
    stack.push(0, 'c');
    stack.join_top();
    stack.shrink_top(1);

    stack.push(0, 'd');
    stack.join_top();
    CHECK(to_vector(stack.top().items) == std::vector{'b', 'd'});
}
