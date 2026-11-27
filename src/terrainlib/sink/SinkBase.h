#pragma once

#include "error/Result.h"

// A sink to derive from, which only has to implement `push`.
template <typename T, typename... Es>
class SinkBase {
public:
    virtual ~SinkBase() = default;

    virtual alp::Result<void, Es...> push(T value) = 0;

    virtual alp::Result<void, Es...> finish() && {
        return {};
    }
};
