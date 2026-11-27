# *****************************************************************************
# AlpineMaps.org
# Copyright (C) 2023 Adam Celarek-Litofcenko
# Copyright (C) 2023 Martin Braunsperger
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
# *****************************************************************************

if(NOT TARGET Catch2::Catch2)
    alp_add_git_repository(catch2 URL https://github.com/catchorg/Catch2.git COMMITISH v3.8.1)
endif()

function(atb_configure_test target)
    target_compile_definitions(${target} PRIVATE
        ALP_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data/"
        ALP_TEST_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/output/"
        ALP_UNITTESTS_EXTENDED=$<BOOL:${ALP_UNITTESTS_EXTENDED}>
        ALP_UNITTESTS_DEBUG_IMAGES=$<BOOL:${ALP_UNITTESTS_DEBUG_IMAGES}>
    )
endfunction()

add_executable(unittests_terrainlib
    assertion_helpers.h
    catch2_helpers.h
    opencv_helpers.h
    result_helpers.h
    temp_directory.h
    throwing_assertions.cpp
    tracked.h

    terrainlib/cli_support_octree_id.cpp
    terrainlib/cli_support_stop_signal.cpp
    terrainlib/convert.cpp
    terrainlib/cow.cpp
    terrainlib/enumerate.cpp
    terrainlib/error.cpp
    terrainlib/error_error.cpp
    terrainlib/error_error_code.cpp
    terrainlib/error_fallible_fn.cpp
    terrainlib/error_format.cpp
    terrainlib/error_result.cpp
    terrainlib/error_result_fn.cpp
    terrainlib/error_test_utils.h
    terrainlib/error_trace.cpp
    terrainlib/error_try.cpp
    terrainlib/error_unwrap.cpp
    terrainlib/error_utils.cpp
    terrainlib/expected_utils.cpp
    terrainlib/file_error.cpp
    terrainlib/fixed_vector.cpp
    terrainlib/fn_sink.cpp
    terrainlib/geometry_bounds.cpp
    terrainlib/geometry_compare.cpp
    terrainlib/geometry_frames.cpp
    terrainlib/gis_dataset.cpp
    terrainlib/gis_dataset_reading.cpp
    terrainlib/gis_error.cpp
    terrainlib/gis_geo_transform.cpp
    terrainlib/gis_pixel_transform.cpp
    terrainlib/gis_srs_bounds.cpp
    terrainlib/gis_srs_classify.cpp
    terrainlib/gis_srs_coverage.cpp
    terrainlib/gis_srs_detail_benchmark.cpp
    terrainlib/gis_srs_geographic.cpp
    terrainlib/gis_srs_system.cpp
    terrainlib/gis_srs_thread_local_system.cpp
    terrainlib/gis_srs_transform.cpp
    terrainlib/gis_srs_unit.cpp
    terrainlib/gis_transform.cpp
    terrainlib/gis_vector_mask.cpp
    terrainlib/glm_serialization.cpp
    terrainlib/glm_utils.cpp
    terrainlib/hash_utils.cpp
    terrainlib/hierarchy_index_map.cpp
    terrainlib/hierarchy_traverse.cpp
    terrainlib/hybrid_index_pair_map.cpp
    terrainlib/hybrid_vector.cpp
    terrainlib/io.cpp
    terrainlib/io_alloc.cpp
    terrainlib/io_bytes.cpp
    terrainlib/io_compression.cpp
    terrainlib/io_conversion.cpp
    terrainlib/io_envelope.cpp
    terrainlib/io_filesystem.cpp
    terrainlib/io_filesystem_utils.cpp
    terrainlib/io_hash.cpp
    terrainlib/io_image.cpp
    terrainlib/io_json_file.cpp
    terrainlib/io_json_serde.cpp
    terrainlib/io_json_value.cpp
    terrainlib/io_limits.cpp
    terrainlib/io_mat.cpp
    terrainlib/io_scratch_directory.cpp
    terrainlib/io_text.cpp
    terrainlib/mem.cpp
    terrainlib/merging_sink.cpp
    terrainlib/mesh_boundary.cpp
    terrainlib/mesh_bounds.cpp
    terrainlib/mesh_cleanup.cpp
    terrainlib/mesh_clip.cpp
    terrainlib/mesh_codec.cpp
    terrainlib/mesh_combine.cpp
    terrainlib/mesh_connected_components.cpp
    terrainlib/mesh_encode.cpp
    terrainlib/mesh_holes.cpp
    terrainlib/mesh_io.cpp
    terrainlib/mesh_io_sf_mesh.cpp
    terrainlib/mesh_manifold.cpp
    terrainlib/mesh_merge.cpp
    terrainlib/mesh_merge_mapping.cpp
    terrainlib/mesh_primitives.cpp
    terrainlib/mesh_reindex.cpp
    terrainlib/mesh_split.cpp
    terrainlib/mesh_storage.cpp
    terrainlib/mesh_topology.cpp
    terrainlib/mesh_validate.cpp
    terrainlib/numeric_checked.cpp
    terrainlib/numeric_checked_cast.cpp
    terrainlib/numeric_overflow.cpp
    terrainlib/numeric_saturating.cpp
    terrainlib/octree_id.cpp
    terrainlib/octree_odd_level_shifted.cpp
    terrainlib/octree_path_layout.cpp
    terrainlib/octree_space.cpp
    terrainlib/octree_storage.cpp
    terrainlib/octree_storage_index_format.cpp
    terrainlib/offset_table.cpp
    terrainlib/once_cell.cpp
    terrainlib/parallel.cpp
    terrainlib/polygon_clip.cpp
    terrainlib/polygon_triangulate.cpp
    terrainlib/progress_indicator_test.cpp
    terrainlib/quantize.cpp
    terrainlib/radix_serialization.cpp
    terrainlib/range.cpp
    terrainlib/range_utils.cpp
    terrainlib/raster_algorithm.cpp
    terrainlib/raster_fold.cpp
    terrainlib/raster_store_attribution_table.cpp
    terrainlib/raster_store_attribution_table_file.cpp
    terrainlib/raster_store_io_float_predictor.cpp
    terrainlib/raster_store_io_tile_codec.cpp
    terrainlib/raster_store_io_tile_codec_benchmark.cpp
    terrainlib/raster_store_path_layout.cpp
    terrainlib/raster_store_storage.cpp
    terrainlib/raster_store_storage_index_format.cpp
    terrainlib/raster_store_storage_metadata.cpp
    terrainlib/raster_store_store_traits.cpp
    terrainlib/raster_store_test_utils.h
    terrainlib/raster_tile_pool.cpp
    terrainlib/raster_view.cpp
    terrainlib/segmented_buffer.cpp
    terrainlib/sf_publish.cpp
    terrainlib/sf_validate_index.cpp
    terrainlib/sink.cpp
    terrainlib/spatial_lookup.cpp
    terrainlib/spsc_channel.cpp
    terrainlib/stamp_set.cpp
    terrainlib/std_compat.cpp
    terrainlib/store_checkpoint.cpp
    terrainlib/store_codec.cpp
    terrainlib/store_draft.cpp
    terrainlib/store_fingerprint.cpp
    terrainlib/store_index.cpp
    terrainlib/store_inputs.cpp
    terrainlib/store_open.cpp
    terrainlib/store_persisted_index.cpp
    terrainlib/store_publish.cpp
    terrainlib/store_raw_storage.cpp
    terrainlib/store_storage.cpp
    terrainlib/store_test_utils.h
    terrainlib/string_utils.cpp
    terrainlib/texture_dilate_colors.cpp
    terrainlib/tiny_vector.cpp
    terrainlib/type_utils.cpp
    terrainlib/union_find.cpp
    terrainlib/unique_resource.cpp
    terrainlib/uv_atlas.cpp
    terrainlib/zpp_bits_utils.cpp
)
# io_image.cpp creates CMYK JPEG images with TurboJPEG.
target_link_libraries(unittests_terrainlib PRIVATE terrainlib Catch2::Catch2WithMain libjpeg-turbo::turbojpeg-static)
atb_configure_test(unittests_terrainlib)

# Keep the aligned GLM configuration isolated from terrainlib and its PCH.
add_executable(unittests_glm_serialization_aligned terrainlib/glm_serialization.cpp)
target_compile_features(unittests_glm_serialization_aligned PRIVATE cxx_std_23)
target_compile_definitions(unittests_glm_serialization_aligned PRIVATE GLM_FORCE_INTRINSICS GLM_FORCE_ALIGNED_GENTYPES)
target_include_directories(unittests_glm_serialization_aligned PRIVATE ${PROJECT_SOURCE_DIR}/src/terrainlib)
target_link_libraries(unittests_glm_serialization_aligned PRIVATE glm::glm zpp_bits Catch2::Catch2WithMain)

if(TARGET sfbuilderlib)
    add_executable(unittests_sfbuilder
        assertion_helpers.h
        catch2_helpers.h
        opencv_helpers.h
        result_helpers.h
        temp_directory.h
        throwing_assertions.cpp
        sf_builder/mesh.cpp
        sf_builder/native_regions.cpp
        sf_builder/texture.cpp
        sf_builder/build.cpp
        sf_builder/test_raster.h
        sf_builder/tile_provider.cpp
    )
    target_link_libraries(unittests_sfbuilder PRIVATE sfbuilderlib Catch2::Catch2WithMain)
    atb_configure_test(unittests_sfbuilder)
endif()

if(TARGET tile-downloader)
    find_package(CURL REQUIRED)
    add_executable(unittests_tile_downloader
        assertion_helpers.h
        result_helpers.h
        temp_directory.h
        throwing_assertions.cpp
        tile_downloader/downloader.cpp
        tile_downloader/http_client.cpp
        tile_downloader/logger.cpp
        tile_downloader/url_builder.cpp
    )
    target_include_directories(unittests_tile_downloader PRIVATE ${PROJECT_SOURCE_DIR}/src/tile_downloader)
    target_link_libraries(unittests_tile_downloader PRIVATE terrainlib Catch2::Catch2WithMain CURL::libcurl)
    atb_configure_test(unittests_tile_downloader)
endif()

if(TARGET dagbuilderlib)
    add_executable(unittests_dagbuilder
        catch2_helpers.h
        temp_directory.h
        throwing_assertions.cpp
        dag_builder/bake_cluster_texture.cpp
        dag_builder/build.cpp
        dag_builder/codec.cpp
        dag_builder/correspondence.cpp
        dag_builder/dag_validate.cpp
        dag_builder/fixed_point.cpp
        dag_builder/merge_clusterings.cpp
        dag_builder/multi_component.cpp
        dag_builder/pull_reproject_texture.cpp
        dag_builder/reprojection.cpp
        dag_builder/slice.cpp
        dag_builder/storage.cpp
        dag_builder/texture_reuse.cpp
        dag_builder/texture_sizing.cpp
        dag_builder/validate.cpp
        dag_builder/weld.cpp
    )
    target_link_libraries(unittests_dagbuilder PUBLIC dagbuilderlib Catch2::Catch2WithMain)
    atb_configure_test(unittests_dagbuilder)
endif()

if(TARGET bvhbuilderlib)
    add_executable(unittests_bvhbuilder
        catch2_helpers.h
        throwing_assertions.cpp
        temp_directory.h
        bvh_builder/frontier_helpers.h
        bvh_builder/admission_gate.cpp
        bvh_builder/batch.cpp
        bvh_builder/batch_cache.cpp
        bvh_builder/batch_reader.cpp
        bvh_builder/batch_storage.cpp
        bvh_builder/bounded_cache.cpp
        bvh_builder/frontier_build.cpp
        bvh_builder/chunk_merger.cpp
        bvh_builder/cut_rule.cpp
        bvh_builder/find_root.cpp
        bvh_builder/flat_traverse.cpp
        bvh_builder/flatten_dag.cpp
        bvh_builder/frustum.cpp
        bvh_builder/json_glm.cpp
        bvh_builder/json_octree.cpp
        bvh_builder/json_serialize.cpp
        bvh_builder/pseudo_lbvh_build.cpp
        bvh_builder/merge_cost.cpp
        bvh_builder/merge_runs.cpp
        bvh_builder/morton_utils.cpp
        bvh_builder/nearest_search.cpp
        bvh_builder/node_cache.cpp
        bvh_builder/gated_ploc_build.cpp
        bvh_builder/hploc_build.cpp
        bvh_builder/prefix_map.cpp
        bvh_builder/row_buffer.cpp
        bvh_builder/run_round.cpp
        bvh_builder/segment_stack.cpp
        bvh_builder/select_cut.cpp
        bvh_builder/sort_by_morton.cpp
        bvh_builder/sort_frontier.cpp
        bvh_builder/split_into_sorted_runs.cpp
        bvh_builder/tabulate_splits.cpp
        bvh_builder/traverse.cpp
        bvh_builder/visit_dag.cpp
        bvh_builder/traverse_tree.cpp
        bvh_builder/widen_build.cpp
        bvh_builder/write_wide_nodes.cpp
    )
    target_link_libraries(unittests_bvhbuilder PUBLIC bvhbuilderlib Catch2::Catch2WithMain)
    atb_configure_test(unittests_bvhbuilder)
endif()

if(TARGET sfmergerlib)
    add_executable(unittests_sfmerger
        assertion_helpers.h
        catch2_helpers.h
        result_helpers.h
        temp_directory.h
        throwing_assertions.cpp
        sf_merger/cut.cpp
        sf_merger/mask.cpp
        sf_merger/merge.cpp
        sf_merger/sphere_projector.cpp
        sf_merger/utils.cpp
    )
    target_link_libraries(unittests_sfmerger PRIVATE sfmergerlib Catch2::Catch2WithMain)
    if(TARGET sf-builder AND TARGET sf-merger)
        target_sources(unittests_sfmerger PRIVATE sf_merger/integration.cpp)
        target_compile_definitions(unittests_sfmerger PRIVATE
            ALP_SF_BUILDER_PATH="$<TARGET_FILE:sf-builder>"
            ALP_SF_MERGER_PATH="$<TARGET_FILE:sf-merger>"
        )
        add_dependencies(unittests_sfmerger sf-builder sf-merger)
    endif()
    atb_configure_test(unittests_sfmerger)
endif()
