#pragma once

#include <memory>
#include <utility>

#include <libassert/assert.hpp>
#include <radix/geometry.h>

#include "dag_id.h"
#include "metadata.h"

namespace dag {

class Cluster {
public:
    Cluster(ClusterId id, std::shared_ptr<const NodeMetadata> batch_metadata)
        : _id(id), _batch_metadata(std::move(batch_metadata)) {
        DEBUG_ASSERT(this->_id.cluster_index < this->_batch_metadata->clusters.size());
    }

    const ClusterId &id() const {
        return this->_id;
    }

    double error() const {
        return get_lod_metrics_for_cluster(*this->_batch_metadata, this->_id.cluster_index).error;
    }

    radix::geometry::Aabb3d lod_bounds() const {
        return get_lod_metrics_for_cluster(*this->_batch_metadata, this->_id.cluster_index).bounds;
    }

    const radix::geometry::Aabb3d &tight_bounds() const {
        return this->metadata().bounds;
    }

private:
    const ClusterMetadata &metadata() const {
        return this->_batch_metadata->clusters[this->_id.cluster_index];
    }

    ClusterId _id;
    std::shared_ptr<const NodeMetadata> _batch_metadata;
};

} // namespace dag
