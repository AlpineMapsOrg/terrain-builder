/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2024 Martin Braunsperger
 * Copyright (C) 2024 Adam Celarek-Litofcenko
 * Copyright (C) 2026 Martin Braunsperger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *****************************************************************************/

#pragma once

#include <filesystem>
#include <optional>
#include <stop_token>
#include <string>
#include <utility>

#include "NodeLoader.h"
#include "NodeWriter.h"
#include "error/Result.h"
#include "error/log.h"
#include "error/try.h"
#include "hierarchy/NodeStatusOrMissing.h"
#include "inputs.h"
#include "mask.h"
#include "merge/NodeData.h"
#include "merge/Result.h"
#include "merge/visitor/Masked.h"
#include "merge/visitor/Simple.h"
#include "merge/visitor/Visitor.h"
#include "mesh/storage.h"
#include "numeric/number_types.h"
#include "octree/Id.h"
#include "sf/publish.h"
#include "sf/validate_index.h"
#include "stop.h"
#include "store/Checkpoint.h"
#include "store/inputs.h"

inline std::string get_dataset_name(const mesh::storage::Storage &storage) {
    return storage.resolver().base_path().filename().string();
}

using MergeNodeError = alp::Error<Stopped, mesh::storage::Draft::SaveError, NodeWriter::LinkSubtreeError>;

template <merge::Visitor Visitor>
class Merger {
public:
    using Status = hierarchy::NodeStatusOrMissing;
    using Context = Visitor::Context;
    using Result = merge::Result<Context>;

    Merger(Visitor& visitor,
        NodeLoader left,
        NodeLoader right,
        NodeWriter output,
        const mesh::storage::Checkpoint& checkpoint,
        std::stop_token stop = {})
        : _visitor(visitor)
        , _left(left)
        , _right(right)
        , _output(std::move(output))
        , _checkpoint(checkpoint)
        , _stop(std::move(stop))
    {
    }

    [[nodiscard]]
    alp::Result<void, MergeNodeError> merge_root() {
        const octree::Id id = octree::Id::root();
        const Context ctx = this->_visitor.make_root_context();
        return merge_node(id, ctx);
    }

    [[nodiscard]]
    alp::Result<void, MergeNodeError> merge_node(const octree::Id &id, const Context& ctx) {
        const Status left_status = this->_left.get_status(id);
        const Status right_status = this->_right.get_status(id);
        return this->merge_node(id, left_status, right_status, ctx);
    }

    [[nodiscard]]
    alp::Result<void, MergeNodeError> merge_node(
        const octree::Id &id,
        const Status left_status,
        const Status right_status,
        const Context& ctx
    ) {
        LOG_DEBUG("[{}] Start merging (left = {}, right = {})", id, left_status, right_status);
        ALP_TRY(check_not_stopped(this->_stop));
        if (this->_checkpoint.has(id)) {
            LOG_DEBUG("[{}] Linked from the checkpoint", id);
            ALP_TRY(this->_output.link_node(id, this->_checkpoint));
            return {};
        }

        const auto merge_result = this->call_merge(id, left_status, right_status, ctx);
        return std::visit([&](const auto &result) -> alp::Result<void, MergeNodeError> {
            using Result = std::decay_t<decltype(result)>;
            if constexpr (std::is_same_v<Result, merge::Recurse<Context>>) {
                LOG_DEBUG("[{}] needs recursion", id);
                const auto children = *ASSERT_VAL(id.children());
                for (const auto &child_id : children) {
                    auto child_result = this->merge_node(child_id, result.context);
                    if (!child_result) {
                        return child_result;
                    }
                }
                return {};
            } else if constexpr (std::is_same_v<Result, merge::Unchanged>) {
                LOG_DEBUG("[{}] remains unchanged (same as {})", id, (result.source == merge::Source::Left ? "left" : "right"));
                // If the node should remain unchanged, but its not present on disk we cannot link it.
                if (result.source == merge::Source::Left && left_status == Status::Missing) {
                    auto mesh_opt = this->_left.load_node(id);
                    if (mesh_opt.has_value()) {
                        auto write_result = this->_output.write_node(id, *mesh_opt);
                        if (!write_result) {
                            return write_result;
                        }
                    }
                } else if (result.source == merge::Source::Right && right_status == Status::Missing) {
                    auto mesh_opt = this->_right.load_node(id);
                    if (mesh_opt.has_value()) {
                        auto write_result = this->_output.write_node(id, *mesh_opt);
                        if (!write_result) {
                            return write_result;
                        }
                    }
                } else {
                    auto link_result = this->_output.link_subtree_to_output(
                        id,
                        result.source == merge::Source::Left ? this->_left : this->_right,
                        this->_stop);
                    if (!link_result) {
                        return link_result;
                    }
                }
                return {};
            } else if constexpr (std::is_same_v<Result, merge::Merged>) {
                LOG_DEBUG("[{}] was merged", id);
                auto write_result = this->_output.write_node(id, result.mesh);
                if (!write_result) {
                    return write_result;
                }
                return {};
            } else if constexpr (std::is_same_v<Result, merge::Ignore>) {
                LOG_DEBUG("[{}] was ignored", id);
                return {};
            }
        }, merge_result);
    }

private:
    static constexpr auto STATUSES = magic_enum::enum_values<Status::Value>();
    static constexpr auto _STATUS_COUNT = STATUSES.size();
    static constexpr auto _MAX_INDEX = _STATUS_COUNT * _STATUS_COUNT;
    using Index = uint_for_max_t<_MAX_INDEX>;
    static constexpr Index STATUS_COUNT = static_cast<Index>(_STATUS_COUNT);
    static constexpr Index MAX_INDEX = static_cast<Index>(_MAX_INDEX);

    static constexpr Index get_index(Status left, Status right) {
        return static_cast<Index>(left) * STATUS_COUNT + static_cast<Index>(right);
    }

    Result call_merge(
        const octree::Id &id,
        Status left,
        Status right,
        const Context& ctx) {
        const Index index = get_index(left, right);

        switch (index) {
#define ALP_GENERATE_CASE(left_status, right_status)                        \
    case get_index(Status::left_status, Status::right_status): {            \
        merge::NodeData<Status::left_status> left_node(id, this->_left);    \
        merge::NodeData<Status::right_status> right_node(id, this->_right); \
        return this->_visitor.template visit<Status::left_status, Status::right_status>(id, left_node, right_node, ctx); \
    }

            ALP_GENERATE_CASE(Missing, Missing);
            ALP_GENERATE_CASE(Missing, Leaf);
            ALP_GENERATE_CASE(Missing, Virtual);

            ALP_GENERATE_CASE(Leaf, Missing);
            ALP_GENERATE_CASE(Leaf, Leaf);
            ALP_GENERATE_CASE(Leaf, Virtual);

            ALP_GENERATE_CASE(Virtual, Missing);
            ALP_GENERATE_CASE(Virtual, Leaf);
            ALP_GENERATE_CASE(Virtual, Virtual);
#undef ALP_GENERATE_CASE

        default:
            UNREACHABLE();
        }
    }

    Visitor& _visitor;
    NodeLoader _left;
    NodeLoader _right;
    NodeWriter _output;
    const mesh::storage::Checkpoint& _checkpoint;
    std::stop_token _stop;
};

using MergeDatasetsError = alp::Error<sf::UnexpectedInnerNode, MergeNodeError>;
// Merges into output_dataset, linking nodes that the checkpoint has instead of merging them.
// Once a stop is requested through `stop`, the merge stops before the next node and fails with Stopped.
// On failure, the output index is saved, so that the draft can serve a later merge as a checkpoint.
[[nodiscard]]
inline alp::Result<void, MergeDatasetsError> merge_datasets(const mesh::storage::Storage& left_dataset,
    const mesh::storage::Storage& right_dataset,
    mesh::storage::Draft& output_dataset,
    const std::optional<MeshMask> mask = std::nullopt,
    const mesh::storage::Checkpoint& checkpoint = {},
    const std::stop_token& stop = {})
{
    for (const mesh::storage::Storage *input : {&left_dataset, &right_dataset}) {
        ALP_TRY_CONTEXT(sf::validate_index(input->index()), alp::make_context("checking the merge input {}", input->base_path()));
    }

    LOG_TRACE("Merging {} and {} into {}",
        get_dataset_name(left_dataset),
        get_dataset_name(right_dataset),
        output_dataset.base_path());

    octree::Space space = octree::Space::earth();
    NodeLoader left(left_dataset, space);
    NodeLoader right(right_dataset, space);
    NodeWriter output(output_dataset);
    alp::Result<void, MergeNodeError> result;
    if (mask.has_value()) {
        merge::visitor::Masked visitor {mask.value(), space};
        Merger<merge::visitor::Masked> merger(visitor, left, right, std::move(output), checkpoint, stop);
        result = merger.merge_root();
    } else {
        merge::visitor::Simple visitor;
        Merger<merge::visitor::Simple> merger(visitor, left, right, std::move(output), checkpoint, stop);
        result = merger.merge_root();
    }
    if (!result.has_value()) {
        LOG_WARN_IF_ERROR(output_dataset.save_index(), "saving the index of the interrupted merge's draft");
    }
    ALP_TRY(std::move(result));
    return {};
}

using MergeError = alp::Error<io::FileError, store::inputs::ValidateReuseError, mesh::storage::OpenError, mesh::storage::CreateError, store::inputs::WriteError, MergeDatasetsError, sf::PublishError>;
// Merges into a new SF storage at `output_path`, reusing the nodes of the draft at `checkpoint_path`, which must have been produced from `inputs`.
[[nodiscard]]
inline alp::Result<void, MergeError> merge_datasets(const mesh::storage::Storage& left_dataset,
    const mesh::storage::Storage& right_dataset,
    const std::filesystem::path& output_path,
    const std::optional<MeshMask>& mask,
    const sf_merger::inputs::Merge& inputs,
    const std::optional<std::filesystem::path>& checkpoint_path = std::nullopt,
    const std::stop_token& stop = {})
{
    const auto context = alp::make_context("merging the SF storages at {} and {} into {}", left_dataset.base_path(), right_dataset.base_path(), output_path);
    for (const mesh::storage::Storage* input : { &left_dataset, &right_dataset }) {
        ALP_TRY_CONTEXT(store::inputs::validate_linkable(input->base_path(), output_path), context);
    }
    mesh::storage::Checkpoint checkpoint;
    if (checkpoint_path.has_value()) {
        auto open_result = store::open_for_reuse<sf_merger::inputs::Schema>(checkpoint_path.value(), output_path, inputs, mesh::storage::open);
        ALP_TRY_CONTEXT(checkpoint, std::move(open_result), context);
    }

    ALP_TRY_CONTEXT(mesh::storage::Draft output, mesh::storage::create(output_path, left_dataset.resolver().layout(), left_dataset.codec_id()), context);
    ALP_TRY_CONTEXT(store::inputs::write<sf_merger::inputs::Schema>(output.base_path(), inputs), context);

    ALP_TRY_CONTEXT(merge_datasets(left_dataset, right_dataset, output, mask, checkpoint, stop), context);
    ALP_TRY_CONTEXT(sf::publish(std::move(output)), context);
    return {};
}
