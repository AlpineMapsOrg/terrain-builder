#pragma once

#include <expected>
#include <functional>
#include <type_traits>
#include <utility>

#include "error/Result.h"
#include "type_utils.h"

namespace alp {

// `F`, called so that it always returns a result with the error `E`.
// The value is whatever `F` returns, and an error `F` returns is converted into `E`.
template <typename E, typename F>
class ResultFn {
public:
    explicit ResultFn(F f) : _f(f) {}

    template <typename... Args>
    [[nodiscard]]
    auto operator()(Args &&...args) {
        using Return = std::remove_cvref_t<std::invoke_result_t<F, Args...>>;

        if constexpr (std::is_void_v<Return>) {
            std::invoke(this->_f, std::forward<Args>(args)...);
            return Result<void, E>();
        } else if constexpr (is_specialization_of_v<Return, std::expected>) {
            return Result<typename Return::value_type, E>(std::invoke(this->_f, std::forward<Args>(args)...));
        } else {
            return Result<Return, E>(std::invoke(this->_f, std::forward<Args>(args)...));
        }
    }

private:
    F _f;
};

} // namespace alp
