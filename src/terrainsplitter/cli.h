#ifndef CLI_H
#define CLI_H

#include <filesystem>
#include <vector>
#include <optional>
#include <span>
#include <string>
#include <string_view>

#include <spdlog/spdlog.h>

namespace cli {
    struct Args {
        std::filesystem::path output_path;
        std::filesystem::path input_path;
        spdlog::level::level_enum log_level;
    };

    Args parse(int argc, const char *const *argv);

    namespace {
    template <typename T, std::size_t Extent>
    Args _parse(const std::span<const T, Extent> args) {
        std::array<const char *, Extent> raw_args;
        std::transform(args.begin(), args.end(), raw_args.begin(),
                       [](const T &str) { return str.data(); });
        return cli::parse(raw_args.size(), raw_args.data());
    }
    template <typename T>
    Args _parse(const std::span<const T, std::dynamic_extent> args) {
        std::vector<const char *> raw_args;
        raw_args.resize(args.size());
        std::transform(args.begin(), args.end(), raw_args.begin(),
                        [](const T &str) { return str.data(); });
        return cli::parse(raw_args.size(), raw_args.data());
    }
    } // namespace

    template <std::size_t Extent>
    Args cli::parse(const std::span<const std::string, Extent> args) {
        return _parse(args);
    }
    template <std::size_t Extent>
    Args cli::parse(const std::span<const std::string_view, Extent> args) {
        return _parse(args);
    }
}

#endif
