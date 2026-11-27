#pragma once

#include <concepts>
#include <functional>
#include <type_traits>
#include <utility>

#include "error/Result.h"
#include "error/ResultFn.h"
#include "error/try.h"

namespace visitor {

template <class E, class F>
class FallibleVisitor {
public:
    FallibleVisitor(F visitor) : _visitor(visitor) {}

    template <class... Args>
    [[nodiscard]]
    alp::Result<bool, E> operator()(Args &&...args) {
        auto result = this->_visitor(std::forward<Args>(args)...);
        using Value = typename decltype(result)::value_type;

        if constexpr (std::same_as<Value, void>) {
            ALP_TRY(std::move(result));
            return true;
        } else {
            static_assert(std::same_as<Value, bool>, "fallible visitor must return void, bool, or a result of either");
            return result;
        }
    }

private:
    alp::ResultFn<F, E> _visitor;
};

} // namespace visitor
