#pragma once

#include <memory>
#include <span>
#include <utility>

#include <libassert/assert.hpp>
#include <radix/geometry.h>

#include "dag_id.h"
#include "metadata.h"

namespace dag {

class Group {
public:
    Group(GroupId id, std::shared_ptr<const NodeMetadata> batch_metadata)
        : _id(id), _batch_metadata(std::move(batch_metadata)) {
        DEBUG_ASSERT(this->_id.group_index < this->_batch_metadata->groups.size());
    }

    const GroupId &id() const {
        return this->_id;
    }

    double error() const {
        return this->metadata().error;
    }

    const radix::geometry::Aabb3d &lod_bounds() const {
        return this->metadata().bounds;
    }

    // Its child clusters.
    std::span<const ClusterId> cluster_ids() const {
        return this->metadata().children;
    }

private:
    const GroupMetadata &metadata() const {
        return this->_batch_metadata->groups[this->_id.group_index];
    }

    GroupId _id;
    std::shared_ptr<const NodeMetadata> _batch_metadata;
};

} // namespace dag
