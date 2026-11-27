#pragma once

#include "error/Infallible.h"
#include "error/Result.h"

// A sink that drops every value.
struct VoidSink {
    template <typename T>
    alp::Result<void, alp::Infallible> push(T &&) {
        return {};
    }

    alp::Result<void, alp::Infallible> finish() && {
        return {};
    }
};
