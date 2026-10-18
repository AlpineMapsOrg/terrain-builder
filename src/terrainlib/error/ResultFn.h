#pragma once

#include <expected>
#include <functional>
#include <type_traits>
#include <utility>

#include "error/Error.h"
#include "error/Infallible.h"
#include "error/Result.h"
#include "type_utils.h"

namespace alp {

namespace detail {

// The error of a result `R`, alp::Infallible for anything else.
template <typename R>
struct ErrorOf {
    using type = Infallible;
};

template <typename T, typename E>
struct ErrorOf<std::expected<T, E>> {
    using type = E;
};

} // namespace detail

// Calls `f` with `args` and returns its outcome as a result, whether `f` returns void, a plain value or a result.
// The error has the codes `Es`, or is `f`'s own without them.
template <typename... Es, typename F, typename... Args>
[[nodiscard]]
auto invoke_fallible(F &&f, Args &&...args) {
    using Result = std::remove_cvref_t<std::invoke_result_t<F, Args...>>;
    using E = std::conditional_t<sizeof...(Es) == 0, typename detail::ErrorOf<Result>::type, Error<Es...>>;

    if constexpr (std::is_void_v<Result>) {
        std::invoke(std::forward<F>(f), std::forward<Args>(args)...);
        return alp::Result<void, E>();
    } else if constexpr (is_specialization_of_v<Result, std::expected>) {
        return alp::Result<typename Result::value_type, E>(std::invoke(std::forward<F>(f), std::forward<Args>(args)...));
    } else {
        return alp::Result<Result, E>(std::invoke(std::forward<F>(f), std::forward<Args>(args)...));
    }
}

// `F`, called so that it always returns a result, as invoke_fallible does.
template <typename F, typename... Es>
class ResultFn {
public:
    explicit ResultFn(F f) : _f(f) {}

    template <typename... Args>
    [[nodiscard]]
    auto operator()(Args &&...args) {
        return invoke_fallible<Es...>(this->_f, std::forward<Args>(args)...);
    }

private:
    F _f;
};

// A ResultFn calling `f`, holding a reference to it if it is an lvalue.
template <typename... Es, typename F>
[[nodiscard]]
ResultFn<F, Es...> make_result_fn(F &&f) {
    return ResultFn<F, Es...>(std::forward<F>(f));
}

} // namespace alp
