#include <algorithm>
#include <filesystem>
#include <memory>
#include <optional>
#include <system_error>
#include <unordered_set>
#include <vector>

#include <fmt/format.h>

#include "error/Context.h"
#include "error/log.h"
#include "error/try.h"
#include "error/utils.h"
#include "io/FileError.h"
#include "io/filesystem.h"
#include "io/serialize.h"
#include "log.h"
#include "io/Error.h"
#include "octree/disk/IndexFile.h"
#include "octree/disk/layout/StrategyRegister.h"
#include "octree/storage/IndexedStorage.h"
#include "octree/storage/helpers.h"
#include "octree/storage/defaults.h"
#include "optional_utils.h"
#include "range_utils.h"

namespace octree {

template <typename T, CodecFor<T> Codec>
alp::Result<IndexedStorage_<T, Codec>, OpenError> open_index(const std::filesystem::path &index_path) {
    LOG_TRACE("Opening storage index {}", index_path);
    const auto context = alp::make_context("opening an index from {}", index_path);

    ALP_TRY_CONTEXT(
        disk::v1::IndexFile index_file,
        io::read_from_path<disk::v1::IndexFile>(index_path),
        context);
    LOG_TRACE("Successfully read storage index with {} entries.", index_file.map.size());
    auto index_map = std::move(index_file.map);
    const auto base_path = index_path.parent_path();
    auto layout_strategy = disk::layout::StrategyRegister::instance().create(index_file.layout_strategy_id);
    if (!layout_strategy) {
        return alp::make_error(disk::layout::UnknownLayoutStrategy{.id = index_file.layout_strategy_id}, context);
    }
    disk::Layout layout(base_path, std::move(layout_strategy), index_file.preferred_extension);
    RawStorage_<T, Codec> raw_storage(std::move(layout));
    return IndexedStorage_<T, Codec>(std::move(raw_storage), std::move(index_map));
}

namespace detail {

// Opens the index of the storage at `base_path`, or nothing when it has no index file.
template <typename T, CodecFor<T> Codec>
alp::Result<std::optional<IndexedStorage_<T, Codec>>, OpenError> open_index_if_present(const std::filesystem::path &base_path) {
    auto storage_result = open_index<T, Codec>(base_path / disk::v1::index_file_name());
    if (storage_result.has_value()) {
        LOG_TRACE("Loaded existing index");
        return std::optional(std::move(storage_result).value());
    }

    if (storage_result.error().template holds_and<io::FileError>(&io::FileError::is_not_found)) {
        return std::nullopt;
    }
    return alp::propagate(std::move(storage_result).error());
}

// The default layout from `options`, falling back to the default strategy and the codec's first extension.
template <typename Codec>
helpers::LayoutWithoutBase make_default_layout(OpenOptions options) {
    std::unique_ptr<disk::layout::Strategy> strategy = options.default_layout_strategy
        ? std::move(options.default_layout_strategy)
        : disk::layout::strategy::make_default();

    const std::vector<std::string_view> supported_extensions = Codec::supported_extensions();
    std::string extension_with_dot = options.preferred_extension_with_dot.value_or(std::string(supported_extensions[0]));
    ASSERT(contains(supported_extensions, extension_with_dot));

    return helpers::LayoutWithoutBase(std::move(strategy), std::move(extension_with_dot));
}

// The layout of the storage at `base_path`, guessed from its files, or the default layout from `options` when it holds none.
template <typename Codec>
alp::Result<helpers::LayoutWithoutBase, io::FileError> resolve_layout(const std::filesystem::path &base_path, OpenOptions options) {
    ALP_TRY(std::optional<helpers::LayoutWithoutBase> guessed_opt, helpers::guess_layout_strategy(base_path));
    if (guessed_opt.has_value()) {
        LOG_TRACE("Guessed layout strategy of dataset as {} and extension as {}",
                  disk::layout::StrategyRegister::instance().get_id(*guessed_opt->strategy),
                  guessed_opt->extension_with_dot);
        return std::move(guessed_opt).value();
    }

    helpers::LayoutWithoutBase layout_info = make_default_layout<Codec>(std::move(options));
    const auto strategy_id = disk::layout::StrategyRegister::instance().get_id(*layout_info.strategy);

    // A dataset that holds nothing yet has no layout to find, so only files that fit no layout are worth a warning.
    ALP_TRY(const bool empty, io::filesystem::is_missing_or_empty_directory(base_path));
    if (empty) {
        LOG_DEBUG("Dataset is empty, using layout strategy {} and extension {}", strategy_id, layout_info.extension_with_dot);
    } else {
        LOG_WARN("Unable to determine layout of dataset, using layout strategy {} and extension {}", strategy_id, layout_info.extension_with_dot);
    }
    return layout_info;
}

// Indexes the nodes stored in `layout`, and saves the index when it holds any.
inline alp::Result<IndexMap, io::Error> create_index_map(const disk::Layout &layout) {
    IndexMap map;
    ALP_TRY(helpers::update_index_map(map, layout));
    if (!map.empty()) {
        ALP_TRY(helpers::save_index_map(map, layout));
    }
    return map;
}

} // namespace detail

template <typename T, CodecFor<T> Codec>
alp::Result<Storage_<T, Codec>, OpenError> open_folder(
    const std::filesystem::path &base_path,
    bool create_index,
    OpenOptions options) {
    LOG_TRACE("Opening storage folder {}", base_path);
    const auto context = alp::make_context("opening storage at {}", base_path);

    ALP_TRY_CONTEXT(io::filesystem::create_directories(base_path), context);

    using IndexedStorageT = IndexedStorage_<T, Codec>;
    ALP_TRY_CONTEXT(std::optional<IndexedStorageT> indexed_opt, (detail::open_index_if_present<T, Codec>(base_path)), context);
    if (indexed_opt.has_value()) {
        return Storage_<T, Codec>(std::move(indexed_opt).value());
    }

    ALP_TRY_CONTEXT(helpers::LayoutWithoutBase layout_info, detail::resolve_layout<Codec>(base_path, std::move(options)), context);
    disk::Layout layout(base_path, std::move(layout_info.strategy), layout_info.extension_with_dot);
    if (!create_index) {
        return Storage_<T, Codec>(RawStorage_<T, Codec>(std::move(layout)));
    }

    ALP_TRY_CONTEXT(IndexMap map, detail::create_index_map(layout), context);
    return Storage_<T, Codec>(RawStorage_<T, Codec>(std::move(layout)), std::move(map));
}

template <typename T, CodecFor<T> Codec>
alp::Result<IndexedStorage_<T, Codec>, OpenError> open_folder_indexed(
    const std::filesystem::path &base_path,
    OpenOptions options) {
    using StorageT = Storage_<T, Codec>;
    ALP_TRY(StorageT storage, (open_folder<T, Codec>(base_path, true, std::move(options))));
    return IndexedStorage_<T, Codec>(std::move(storage));
}

} // namespace octree
