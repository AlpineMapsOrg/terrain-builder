#pragma once

#include <concepts>
#include <expected>
#include <type_traits>

#include "type_utils.h"

namespace alp {

// A callable that takes `Args` and returns a result holding a `T`.
template <typename F, typename T, typename... Args>
concept FallibleFn = std::invocable<F, Args...>
    && is_specialization_of_v<std::remove_cvref_t<std::invoke_result_t<F, Args...>>, std::expected>
    && std::same_as<typename std::remove_cvref_t<std::invoke_result_t<F, Args...>>::value_type, T>;

} // namespace alp
