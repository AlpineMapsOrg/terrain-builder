#include "ImGuiExtensions.h"
#include <imgui.h>
#include <log.h>

bool ImGui::ALP::InputOctreeId(std::optional<octree::Id> &octree_id_out, octree::Id::Level &tmp_octree_zoom, octree::Id::Coords &tmp_octree_coords, octree::Id::Index &tmp_octree_index)
{
    bool edited = false;
    float border_size = octree_id_out.has_value() ? 0.0f : 1.0f;

    ImGui::PushStyleColor(ImGuiCol_Border, (ImVec4)ImColor::HSV(0, 1, 1));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, border_size);

    if (ImGui::InputScalar("Zoom Level", ImGuiDataType_U8, &tmp_octree_zoom))
    {
        // First try with Coords
        octree_id_out = octree::Id::try_make(tmp_octree_zoom, tmp_octree_coords);

        // Then if unsuccessful, try with index
        if (!octree_id_out.has_value())
        {
            octree_id_out = octree::Id::try_make(tmp_octree_zoom, tmp_octree_index);
        }

        // If some of the above was successful, populate the coord and index fields
        if (octree_id_out.has_value())
        {
            tmp_octree_index = octree_id_out.value().index_on_level();
            tmp_octree_coords = octree_id_out.value().coords();
        }

        edited = true;
    }

    if (ImGui::InputScalarN("Coords", ImGuiDataType_U32, &tmp_octree_coords, 3))
    {
        octree_id_out.reset();
        octree_id_out = octree::Id::try_make(tmp_octree_zoom, tmp_octree_coords);

        if (octree_id_out.has_value())
        {
            tmp_octree_index = octree_id_out.value().index_on_level();
        }

        edited = true;
    }

    if (ImGui::InputScalar("Index", ImGuiDataType_U64, &tmp_octree_index))
    {
        octree_id_out.reset();
        octree_id_out = octree::Id::try_make(tmp_octree_zoom, tmp_octree_index);

        if (octree_id_out.has_value())
        {
            tmp_octree_coords = octree_id_out.value().coords();
        }

        edited = true;
    }

    ImGui::PopStyleColor();
    ImGui::PopStyleVar();

    return edited;
}