cmake_minimum_required(VERSION 3.21)

project(alpine-terrain-builder-unittests)

option(ATB_UNITTESTS_EXTENDED "perform extended unit tests (they take long)" OFF)
option(ATB_UNITTESTS_DEBUG_IMAGES "output debug height images for visual comparison" OFF)
set(ATB_UNITTESTS_AUSTRIA_HIGHRES CACHE FILEPATH "path to the high res austrian dataset (terrain model, i.e., OeRect_01m_gt_31287.img)")

include(FetchContent)
FetchContent_Declare(catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.3.2
    SOURCE_DIR 		${CMAKE_SOURCE_DIR}/3rdparty/Catch2
)
FetchContent_MakeAvailable(catch2)

add_executable(unittests_terrainlib
    alpine_raster_format.cpp
    catch2_helpers.h

    dataset.cpp
    dataset_reading.cpp
    depth_first_tile_traverser.cpp
    grid.cpp
    image.cpp
    main.cpp
    parallel_tile_generator.cpp
    parallel_tiler.cpp
    progress_indicator_test.cpp
    srs_test.cpp
    tile_heights_generator.cpp
    top_down_tiler.cpp
    algorithms/primitives.cpp
    algorithms/raster_triangle_scanline.cpp
    tntn/Delaunay_tests.cpp
    tntn/File_tests.cpp
    tntn/geometrix_tests.cpp
    tntn/Mesh_tests.cpp
    tntn/ObjPool_tests.cpp
    tntn/OFFReader_tests.cpp

    tntn/RasterIO_tests.cpp
    tntn/Raster_tests.cpp
    tntn/raster_tools_tests.cpp
    tntn/simple_meshing_tests.cpp
    tntn/SimpleRange_tests.cpp
    tntn/SuperTriangle_tests.cpp
    tntn/SurfacePoints_tests.cpp
    tntn/terra_meshing_tests.cpp
    tntn/test_common.cpp
    tntn/test_common.h
    tntn/triangle_indices.cpp
    tntn/triangle_indices_data.cpp
    tntn/triangle_indices.h
    tntn/util_tests.cpp
    tntn/vertex_points.cpp
    tntn/vertex_points.h)
    
target_compile_definitions(unittests_terrainlib PUBLIC "ATB_TEST_DATA_DIR=\"${CMAKE_SOURCE_DIR}/unittests/data/\"")
target_compile_definitions(unittests_terrainlib PUBLIC "ATB_UNITTESTS_AUSTRIA_HIGHRES=\"${ATB_UNITTESTS_AUSTRIA_HIGHRES}\"")
if (ATB_UNITTESTS_EXTENDED)
    target_compile_definitions(unittests_terrainlib PUBLIC "ATB_UNITTESTS_EXTENDED=true")
else()
    target_compile_definitions(unittests_terrainlib PUBLIC "ATB_UNITTESTS_EXTENDED=false")
endif()
if (ATB_UNITTESTS_DEBUG_IMAGES)
    target_compile_definitions(unittests_terrainlib PUBLIC "ATB_UNITTESTS_DEBUG_IMAGES=true")
else()
    target_compile_definitions(unittests_terrainlib PUBLIC "ATB_UNITTESTS_DEBUG_IMAGES=false")
endif()
target_link_libraries(unittests_terrainlib PUBLIC terrainlib Catch2::Catch2)


add_executable(unittests_terrainmerger
    catch2_helpers.h
    terrainmerger/convert.cpp
    terrainmerger/merge.cpp)

target_link_libraries(unittests_terrainmerger PUBLIC terrainmergerlib Catch2::Catch2)


add_executable(unittests_terrainbuilder
    catch2_helpers.h
    terrainbuilder/mesh_io.cpp
    terrainbuilder/mesh.cpp
    terrainbuilder/texture.cpp)
target_link_libraries(unittests_terrainbuilder PUBLIC terrainbuilderlib terrainmergerlib terrainlib Catch2::Catch2 CGAL::CGAL)
target_compile_definitions(unittests_terrainbuilder PUBLIC "ATB_TEST_DATA_DIR=\"${CMAKE_SOURCE_DIR}/unittests/data/\"")
