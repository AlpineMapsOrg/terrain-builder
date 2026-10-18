#pragma once

#include <utility>

#include "error/ResultFn.h"

// Does nothing when a stream ends.
struct NoFinish {
    void operator()() const {}
};

// A sink that passes each value to `push`, and calls `finish` when its stream ends.
template <typename Push, typename Finish = NoFinish>
class FnSink {
public:
    explicit FnSink(Push push, Finish finish = {})
        : _push(std::move(push)),
          _finish(std::move(finish)) {}

    template <typename V>
    auto push(V &&value) {
        return alp::invoke_fallible(this->_push, std::forward<V>(value));
    }

    auto finish() && {
        return alp::invoke_fallible(this->_finish);
    }

private:
    Push _push;
    Finish _finish;
};
