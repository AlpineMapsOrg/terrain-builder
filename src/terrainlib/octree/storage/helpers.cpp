#include <memory>
#include <optional>
#include <filesystem>
#include <vector>
#include <algorithm>
#include <unordered_set>

#include <fmt/format.h>

#include "octree/storage/helpers.h"
#include "error/Context.h"
#include "error/try.h"
#include "io/filesystem.h"
#include "io/serialize.h"
#include "octree/NodeStatus.h"
#include "octree/IndexMap.h"
#include "octree/disk/IndexFile.h"
#include "octree/disk/Layout.h"
#include "octree/disk/layout/Strategy.h"
#include "octree/disk/layout/StrategyRegister.h"

namespace octree::helpers {
namespace {
template <typename Key, typename Value>
std::optional<Key> find_max_key(const std::unordered_map<Key, Value>& map) {
    if (map.empty()) {
        return std::nullopt;
    }

    auto max_it = map.begin();
    for (auto it = std::next(max_it); it != map.end(); it++) {
        if (it->second > max_it->second) {
            max_it = it;
        }
    }

    return max_it->first;
}
}

alp::Result<std::optional<LayoutWithoutBase>, io::FileError> guess_layout_strategy(
    const std::filesystem::path &base_path,
    size_t max_files_to_check) {

    const auto context = alp::make_context("guessing the layout of storage at {}", base_path);

    ALP_TRY_CONTEXT(const bool is_directory, io::filesystem::is_directory(base_path), context);
    if (!is_directory) {
        return std::nullopt;
    }

    std::vector<std::filesystem::path> candidate_paths;
    std::unordered_map<std::string, size_t> extension_counters;
    const auto visit = [&](const std::filesystem::path &path) {
        if (max_files_to_check == 0) {
            return false;
        }

        const auto ext = path.extension();
        if (ext == disk::v1::index_extension()) {
            return true;
        }
        extension_counters[ext]++;

        candidate_paths.emplace_back(path.lexically_relative(base_path));
        max_files_to_check--;
        return true;
    };
    ALP_TRY_CONTEXT(io::filesystem::visit_files_recursively(base_path, visit), context);
    std::optional<std::string> most_common_ext = find_max_key(extension_counters);

    std::unique_ptr<disk::layout::Strategy> best_strategy;
    size_t best_match_count = 0;
    const auto &factories = disk::layout::StrategyRegister::instance().factories();
    for (const auto &[_, make_strategy] : factories) {
        auto strategy = make_strategy(); // assume returns unique_ptr<Strategy>

        size_t match_count = 0;
        for (const auto &rel_path : candidate_paths) {
            if (strategy->get_id_from_relative_node_path(rel_path).has_value()) {
                match_count++;
            }
        }

        if (match_count > best_match_count) {
            best_match_count = match_count;
            best_strategy = std::move(strategy);
        }
    }

    if (best_strategy && most_common_ext.has_value()) {
        return LayoutWithoutBase(std::move(best_strategy), most_common_ext.value());
    }

    return std::nullopt;
}

io::Result<void> save_index_map(const IndexMap& index, const disk::Layout& layout) {
    const auto index_path = layout.base_path() / disk::v1::index_file_name();
    LOG_TRACE("Saving octree storage index to {}", index_path);

    disk::v1::IndexFile index_file;
    index_file.map = index;
    index_file.preferred_extension = layout.extension_with_dot();
    index_file.layout_strategy_id = disk::layout::StrategyRegister::instance().get_id(layout.strategy());

    ALP_TRY_CONTEXT(io::write_to_path(index_file, index_path), fmt::format("saving an index to {}", index_path));
    return {};
}

alp::Result<void, io::FileError> update_index_map(IndexMap &index, const disk::Layout &layout) {
    index.clear();
    const auto &base_path = layout.base_path();
    const auto context = alp::make_context("indexing the nodes of storage at {}", base_path);

    ALP_TRY_CONTEXT(io::filesystem::create_directories(base_path), context);

    const auto visit = [&](const std::filesystem::path &path) {
        const auto ext = path.extension();
        if (ext == disk::v1::index_extension()) {
            return true;
        }

        auto id_opt = layout.get_id_from_node_path(path);
        if (!id_opt) {
            return true;
        }

        const Id id = *id_opt;
        index.add(id);
        return true;
    };
    ALP_TRY_CONTEXT(io::filesystem::visit_files_recursively(base_path, visit), context);
    return {};
}

}
