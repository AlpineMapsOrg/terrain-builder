#pragma once

#include <libassert/assert.hpp>

#include "NodeLoader.h"
#include "error/unwrap.h"
#include "mesh/SimpleMesh.h"
#include "mesh/validate.h"
#include "octree/Id.h"
#include "octree/NodeStatus.h"
#include "octree/Storage.h"
#include "octree/storage/Storage.h"
#include "octree/traverse.h"
#include "texture/io.h"

// TODO: make thread safe
class NodeWriter {
public:
    NodeWriter(octree::Storage &storage) : _storage(storage) {}

    bool has_node(const octree::Id &id) {
        return this->_storage.has(id);
    }

    void write_node(const octree::Id &id, const SimpleMesh &mesh) {
        mesh::validate(mesh);
        alp::unwrap(this->_storage.save(id, mesh));
        auto p = this->_storage.path_for(id);
        // change extension to .png
        p.replace_extension(".png");
        alp::unwrap(texture::io::write_to_path(mesh.texture.value(), p));
    }

    void copy_subtree_to_output(
        const octree::Id &id,
        const NodeLoader &loader) {
        octree::traverse(
            loader.storage().index(),
            [&](const octree::Id &child_id, const octree::NodeStatus &status) {
                if (status == octree::NodeStatus::Virtual) {
                    return;
                }
                DEBUG_ASSERT(status == octree::NodeStatus::Leaf);

                alp::unwrap(this->_storage.copy_from(child_id, loader.storage()));
            },
            octree::always_refine,
            id);
    }

private:
    octree::Storage &_storage;
};
