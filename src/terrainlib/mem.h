#pragma once

#include <cstddef>
#include <type_traits>
#include <utility>

template <typename T>
constexpr size_t sizeof_val(const T &value) {
    if constexpr (requires { value.size_bytes(); }) {
        return value.size_bytes();
    } else {
        return sizeof(value);
    }
}

template <typename... T>
constexpr size_t sizeof_val(const T &...values) {
    return (sizeof_val(values) + ... + size_t{0});
}

// Destroys `value`, which must be moved in.
template <typename T>
    requires(!std::is_lvalue_reference_v<T>)
void drop(T &&value) {
    [[maybe_unused]] const T dropped(std::move(value));
}
