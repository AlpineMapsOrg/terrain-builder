#pragma once

#include <octree/Id.h>

namespace ImGui::ALP
{
    bool InputOctreeId(std::optional<octree::Id> &octree_id_out, octree::Id::Level &tmp_octree_zoom, octree::Id::Coords &tmp_octree_coords, octree::Id::Index &tmp_octree_index);
}