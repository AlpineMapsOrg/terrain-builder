if(NOT TARGET Catch2::Catch2)
    alp_add_git_repository(catch2 URL https://github.com/catchorg/Catch2.git COMMITISH v3.8.1)
endif()

function(atb_configure_test target)
    target_compile_definitions(${target} PRIVATE
        CATCH_CONFIG_ENABLE_ALL_STRINGMAKERS
        ALP_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/unittests/data/"
        ALP_TEST_OUTPUT_DIR="${CMAKE_BINARY_DIR}/unittests/output/"
        ALP_UNITTESTS_AUSTRIA_HIGHRES="${ALP_UNITTESTS_AUSTRIA_HIGHRES}"
        ALP_UNITTESTS_EXTENDED=$<BOOL:${ALP_UNITTESTS_EXTENDED}>
        ALP_UNITTESTS_DEBUG_IMAGES=$<BOOL:${ALP_UNITTESTS_DEBUG_IMAGES}>
    )
endfunction()

add_executable(unittests_terrainlib
    catch2_helpers.h
    throwing_assertions.cpp
    opencv_helpers.h

    terrainlib/chunked_ring_buffer.cpp
    terrainlib/convert.cpp
    terrainlib/cow.cpp
    terrainlib/enumerate.cpp
    terrainlib/error.cpp
    terrainlib/error_error_code.cpp
    terrainlib/error_fallible_fn.cpp
    terrainlib/error_format.cpp
    terrainlib/error_result.cpp
    terrainlib/error_result_fn.cpp
    terrainlib/error_trace.cpp
    terrainlib/error_try.cpp
    terrainlib/error_unwrap.cpp
    terrainlib/error_utils.cpp
    terrainlib/file_error.cpp
    terrainlib/fixed_vector.cpp
    terrainlib/gdal_dataset.cpp
    terrainlib/gdal_error.cpp
    terrainlib/gdal_pixel_transform.cpp
    terrainlib/geometry_frames.cpp
    terrainlib/grid.cpp
    terrainlib/hash_utils.cpp
    terrainlib/hybrid_index_pair_map.cpp
    terrainlib/hybrid_vector.cpp
    terrainlib/offset_table.cpp
    terrainlib/segmented_buffer.cpp
    terrainlib/stamp_set.cpp
    terrainlib/index_map.cpp
    terrainlib/index_traverse.cpp
    terrainlib/io.cpp
    terrainlib/io_compression.cpp
    terrainlib/io_envelope.cpp
    terrainlib/io_filesystem.cpp
    terrainlib/io_text.cpp
    terrainlib/mem.cpp
    terrainlib/mesh_boundary.cpp
    terrainlib/mesh_bounds.cpp
    terrainlib/mesh_cleanup.cpp
    terrainlib/mesh_clip.cpp
    terrainlib/mesh_combine.cpp
    terrainlib/mesh_connected_components.cpp
    terrainlib/mesh_cut.cpp
    terrainlib/mesh_cut_to_disk.cpp
    terrainlib/mesh_encode.cpp
    terrainlib/mesh_holes.cpp
    terrainlib/mesh_io.cpp
    terrainlib/mesh_manifold.cpp
    terrainlib/mesh_merge_mapping.cpp
    terrainlib/mesh_merge.cpp
    terrainlib/mesh_reindex.cpp
    terrainlib/mesh_split.cpp
    terrainlib/mesh_topology.cpp
    terrainlib/octree_id.cpp
    terrainlib/octree_space.cpp
    terrainlib/octree_odd_level_shifted.cpp
    terrainlib/octree_storage.cpp
    terrainlib/once_cell.cpp
    terrainlib/parallel.cpp
    terrainlib/polygon_clip.cpp
    terrainlib/polygon_triangulate.cpp
    terrainlib/progress_indicator_test.cpp
    terrainlib/quantize.cpp
    terrainlib/range.cpp
    terrainlib/range_utils.cpp
    terrainlib/spatial_lookup.cpp
    terrainlib/spsc_channel.cpp
    terrainlib/srs_shared_system.cpp
    terrainlib/srs_system.cpp
    terrainlib/srs_transform.cpp
    terrainlib/srs_transform_bounds.cpp
    terrainlib/string_utils.cpp
    terrainlib/texture_dilate_colors.cpp
    terrainlib/texture_io.cpp
    terrainlib/tiny_vector.cpp
    terrainlib/type_utils.cpp
    terrainlib/union_find.cpp
    terrainlib/unique_resource.cpp
    terrainlib/uv_atlas.cpp
    terrainlib/zpp_bits_utils.cpp
)
target_link_libraries(unittests_terrainlib PRIVATE terrainlib Catch2::Catch2WithMain)
atb_configure_test(unittests_terrainlib)

if(TARGET tilebuilderlib)
    add_executable(unittests_tilebuilder
        catch2_helpers.h
        throwing_assertions.cpp
        tilebuilder/alpine_raster_format.cpp
        tilebuilder/dataset_reading.cpp
        tilebuilder/depth_first_tile_traverser.cpp
        tilebuilder/image_writer.cpp
        tilebuilder/parallel_tile_generator.cpp
        tilebuilder/parallel_tiler.cpp
        tilebuilder/tile_heights_generator.cpp
        tilebuilder/top_down_tiler.cpp
    )
    target_link_libraries(unittests_tilebuilder PRIVATE tilebuilderlib Catch2::Catch2WithMain)
    atb_configure_test(unittests_tilebuilder)
endif()

if(TARGET sfbuilderlib)
    add_executable(unittests_sfbuilder
        catch2_helpers.h
        throwing_assertions.cpp
        sf_builder/mesh.cpp
        sf_builder/texture.cpp
        sf_builder/tile_provider.cpp
    )
    target_link_libraries(unittests_sfbuilder PRIVATE sfbuilderlib Catch2::Catch2WithMain)
    atb_configure_test(unittests_sfbuilder)
endif()

if(TARGET tile-downloader)
    find_package(CURL REQUIRED)
    add_executable(unittests_tile_downloader
        throwing_assertions.cpp
        tile_downloader/downloader.cpp
        tile_downloader/http_client.cpp
        tile_downloader/logger.cpp
        tile_downloader/url_builder.cpp
        tile_downloader/write_file.cpp
    )
    target_include_directories(unittests_tile_downloader PRIVATE ${CMAKE_SOURCE_DIR}/src/tile_downloader)
    target_link_libraries(unittests_tile_downloader PRIVATE terrainlib Catch2::Catch2WithMain CURL::libcurl)
    atb_configure_test(unittests_tile_downloader)
endif()

if(TARGET dagbuilderlib)
    add_executable(unittests_dagbuilder
        catch2_helpers.h
        throwing_assertions.cpp
        dag_builder/bake_cluster_texture.cpp
        dag_builder/correspondence.cpp
        dag_builder/reprojection.cpp
        dag_builder/merge_clusterings.cpp
        dag_builder/multi_component.cpp
        dag_builder/weld.cpp
        dag_builder/slice.cpp
        dag_builder/pull_reproject_texture.cpp
        dag_builder/fixed_point.cpp
        dag_builder/texture_sizing.cpp
        dag_builder/texture_reuse.cpp
    )
    target_link_libraries(unittests_dagbuilder PUBLIC dagbuilderlib Catch2::Catch2WithMain)
    atb_configure_test(unittests_dagbuilder)
endif()

if(TARGET sfmergerlib)
    add_executable(unittests_sfmerger
        catch2_helpers.h
        throwing_assertions.cpp
        sf_merger/mask.cpp
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
