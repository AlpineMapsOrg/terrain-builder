#pragma once

#include <optional>
#include <functional>
#include <type_traits>

template <typename T>
std::optional<std::reference_wrapper<const T>> as_ref(const std::optional<T> &option) {
    if (option.has_value()) {
        return option.value();
    } else {
        return std::nullopt;
    }
}
template <typename T>
std::optional<std::reference_wrapper<T>> as_ref(std::optional<T> &option) {
    if (option.has_value()) {
        return option.value();
    } else {
        return std::nullopt;
    }
}

template <typename T, typename Func>
auto map(const std::optional<T> &option, Func &&f) -> std::optional<std::invoke_result_t<Func, T>> {
    using U = std::invoke_result_t<Func, T>;

    if (option.has_value()) {
        return std::optional<U>{std::forward<Func>(f)(option.value())};
    } else {
        return std::nullopt;
    }
}

template <typename T, typename DefaultFunc, typename Func>
auto map_or_else(const std::optional<T> &option, DefaultFunc &&default_f, Func &&f) -> std::invoke_result_t<Func, const T &> {
    if (option.has_value()) {
        return std::forward<Func>(f)(option.value());
    } else {
        return std::forward<DefaultFunc>(default_f)();
    }
}

template <typename T, typename Func>
auto map_or(const std::optional<T> &option, std::type_identity_t<std::invoke_result_t<Func, const T &>> default_value, Func &&f) -> std::invoke_result_t<Func, const T &> {
    return map_or_else(option, [&] { return std::move(default_value); }, std::forward<Func>(f));
}

template <typename T, typename Func>
T& get_or_insert_with(std::optional<T> &option, Func &&f) {
    if (!option.has_value()) {
        T value = std::forward<Func>(f)();
        option = value;
    }
    return option.value();
}

template <typename T>
T& get_or_insert(std::optional<T> &option, std::type_identity_t<T> value) {
    return get_or_insert_with(option, [&] { return std::move(value); });
}

template <typename T, typename Func>
T unwrap_or_else(const std::optional<T> &option, Func &&f) {
    if (option.has_value()) {
        return option.value();
    }
    return std::forward<Func>(f)();
}

template <typename T>
T unwrap_or(const std::optional<T> &option, std::type_identity_t<T> value) {
    return unwrap_or_else(option, [&] { return std::move(value); });
}

template <typename T>
std::optional<T> flatten(const std::optional<std::optional<T>> option) {
    if (option.has_value()) {
        return option.value();
    } else {
        return std::nullopt;
    }
}
