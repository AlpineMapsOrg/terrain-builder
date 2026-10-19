#pragma once

#include <concepts>
#include <filesystem>
#include <ranges>

#include "error/Result.h"

template <typename Codec, typename T>
concept CodecFor =
    requires(const std::filesystem::path &path, const T &value) {
        typename Codec::value_type;
        typename Codec::LoadError;
        typename Codec::SaveError;

        requires std::same_as<typename Codec::value_type, T>;

        { Codec::load_from_path(path) }
            -> std::same_as<alp::Result<T, typename Codec::LoadError>>;

        { Codec::save_to_path(value, path) }
            -> std::same_as<alp::Result<void, typename Codec::SaveError>>;

        { Codec::supported_extensions() } -> std::ranges::range;

        requires std::convertible_to<
            std::ranges::range_value_t<decltype(Codec::supported_extensions())>,
            std::string_view>;
    };
