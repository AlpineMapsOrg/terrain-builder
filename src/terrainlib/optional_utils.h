#pragma once

#include <optional>
#include <expected>
#include <functional>
#include <type_traits>
#include <utility>

namespace alp {

template <typename T>
std::optional<std::reference_wrapper<const T>> ref(const std::optional<T> &option) {
    if (option.has_value()) {
        return option.value();
    } else {
        return std::nullopt;
    }
}
template <typename T>
std::optional<std::reference_wrapper<T>> ref(std::optional<T> &option) {
    if (option.has_value()) {
        return option.value();
    } else {
        return std::nullopt;
    }
}

template <typename T, typename DefaultFunc, typename Func>
auto transform_or_else(const std::optional<T> &option, DefaultFunc &&default_f, Func &&f) -> std::remove_cvref_t<std::invoke_result_t<Func, const T &>> {
    if (option.has_value()) {
        return std::invoke(std::forward<Func>(f), option.value());
    } else {
        return std::invoke(std::forward<DefaultFunc>(default_f));
    }
}

template <typename T, typename Func>
auto transform_or(const std::optional<T> &option, std::type_identity_t<std::remove_cvref_t<std::invoke_result_t<Func, const T &>>> default_value, Func &&f) -> std::remove_cvref_t<std::invoke_result_t<Func, const T &>> {
    return transform_or_else(option, [&] { return std::move(default_value); }, std::forward<Func>(f));
}

template <typename T, typename Func>
T& value_or_emplace_else(std::optional<T> &option, Func &&f) {
    if (!option.has_value()) {
        T value = std::invoke(std::forward<Func>(f));
        option = value;
    }
    return option.value();
}

template <typename T>
T& value_or_emplace(std::optional<T> &option, std::type_identity_t<T> value) {
    return value_or_emplace_else(option, [&] { return std::move(value); });
}

template <typename T, typename Func>
T value_or_else(const std::optional<T> &option, Func &&f) {
    if (option.has_value()) {
        return option.value();
    }
    return std::invoke(std::forward<Func>(f));
}

template <typename T, typename Func>
auto to_expected_or_else(std::optional<T> option, Func &&f) -> std::expected<T, std::invoke_result_t<Func>> {
    if (option.has_value()) {
        return std::move(option).value();
    } else {
        return std::unexpected(std::forward<Func>(f)());
    }
}

template <typename T, typename E>
std::expected<T, E> to_expected_or(std::optional<T> option, E error) {
    return to_expected_or_else(std::move(option), [=]() { return std::move(error); });
}

template <typename T>
std::optional<T> flatten(const std::optional<std::optional<T>> option) {
    if (option.has_value()) {
        return option.value();
    } else {
        return std::nullopt;
    }
}

} // namespace alp
