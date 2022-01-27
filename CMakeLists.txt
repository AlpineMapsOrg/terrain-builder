cmake_minimum_required(VERSION 3.18)

project(alpine-terrain-builder)

option(ATB_UNITTESTS "include unit test targets in the buildsystem" ON)
option(ATB_UNITTESTS_EXTENDED "perform extended unit tests (they take long)" ON)
option(ATB_UNITTESTS_DEBUG_IMAGES "output debug height images for visual comparison" OFF)
option(ATB_ENABLE_ADDRESS_SANITIZER "compiles atb with address sanitizer enabled (only debug, works only on g++ and clang)" ON)
option(ATB_ENABLE_THREAD_SANITIZER "compiles atb with thread sanitizer enabled (only debug, works only on g++ and clang)" OFF)
option(ATB_ENABLE_ASSERTS "enable asserts (do not define NDEBUG)" ON)
option(ATB_USE_LLVM_LINKER "use lld (llvm) for linking. it's parallel and much faster, but not installed by default. if it's not installed, you'll get errors, that openmp or other stuff is not installed (hard to track down)" OFF)

set(CMAKE_CXX_STANDARD 20)

find_package(GDAL REQUIRED)
if(NOT GDAL_CONFIG)
    message(FATAL_ERROR "gdal-config command not found (not in PATH?), cannot proceed")
endif()

find_package(TBB REQUIRED)

execute_process(
  COMMAND ${GDAL_CONFIG} --version
  OUTPUT_VARIABLE SYSTEM_GDAL_VERSION
)

if(SYSTEM_GDAL_VERSION VERSION_LESS "3.3")
    message(FATAL_ERROR "GDAL version \"${SYSTEM_GDAL_VERSION}\" is too old, at least 3.3 is required")
endif()

# Boost
#set(Boost_USE_STATIC_LIBS ON CACHE BOOL "" FORCE)
#set(Boost_USE_STATIC_LIBS ON)
#find_package(Boost REQUIRED COMPONENTS
    #program_options
    #filesystem
    #system
#)

find_package(ZLIB REQUIRED)
find_package(fmt REQUIRED)
if (ATB_UNITTESTS)
    find_package(Catch2 REQUIRED)
endif()
set(ATB_INCLUDE_PATHS

)



set(ATB_TERRAINLIB_HEADERS
    terrainlib/alpine_raster.h
    terrainlib/cesium_tin_terra.h
    terrainlib/Dataset.h
    terrainlib/DatasetReader.h
    terrainlib/Exception.h
    terrainlib/Image.h
    terrainlib/layer_json_writer.h
    terrainlib/MetaDataGenerator.h
    terrainlib/ParallelTileGenerator.h
    terrainlib/ProgressIndicator.h
    terrainlib/srs.h
    terrainlib/Tile.h
    terrainlib/Tiler.h
    terrainlib/algorithms/primitives.h
    terrainlib/algorithms/raster_triangle_scanline.h
    terrainlib/ctb/Bounds.hpp
    terrainlib/ctb/Coordinate.hpp
    terrainlib/ctb/Coordinate3D.hpp
    terrainlib/ctb/CTBException.hpp
    terrainlib/ctb/GlobalGeodetic.hpp
    terrainlib/ctb/GlobalMercator.hpp
    terrainlib/ctb/Grid.hpp
    terrainlib/ctb/TileCoordinate.hpp
    terrainlib/ctb/types.hpp
    terrainlib/tntn/BinaryIO.h
    terrainlib/tntn/Delaunator.h
    terrainlib/tntn/DelaunayMesh.h
    terrainlib/tntn/DelaunayTriangle.h
    terrainlib/tntn/endianness.h
    terrainlib/tntn/FileFormat.h
    terrainlib/tntn/File.h
    terrainlib/tntn/gdal_init.h
    terrainlib/tntn/geometrix.h
    terrainlib/tntn/logging.h
    terrainlib/tntn/Mesh2Raster.h
    terrainlib/tntn/Mesh.h
    terrainlib/tntn/MeshIO.h
    terrainlib/tntn/MeshMode.h
    terrainlib/tntn/MeshWriter.h
    terrainlib/tntn/ObjPool.h
    terrainlib/tntn/OFFReader.h
    terrainlib/tntn/Points2Mesh.h
    terrainlib/tntn/QuadEdge.h
    terrainlib/tntn/QuantizedMeshIO.h
    terrainlib/tntn/Raster.h
    terrainlib/tntn/RasterIO.h
    terrainlib/tntn/raster_tools.h
    terrainlib/tntn/simple_meshing.h
    terrainlib/tntn/SuperTriangle.h
    terrainlib/tntn/SurfacePoints.h
    terrainlib/tntn/TerraMesh.h
    terrainlib/tntn/terra_meshing.h
    terrainlib/tntn/TerraUtils.h
    terrainlib/tntn/tntn_assert.h
    terrainlib/tntn/util.h
    terrainlib/tntn/ZemlyaMesh.h
    terrainlib/tntn/zemlya_meshing.h
    terrainlib/tntn/ZoomRange.h

)
set(ATB_TERRAINLIB_SOURCES
    terrainlib/alpine_raster.cpp
    terrainlib/cesium_tin_terra.cpp
    terrainlib/Dataset.cpp
    terrainlib/DatasetReader.cpp
    terrainlib/Image.cpp
    terrainlib/layer_json_writer.cpp
    terrainlib/MetaDataGenerator.cpp
    terrainlib/ParallelTileGenerator.cpp
    terrainlib/ProgressIndicator.cpp
    terrainlib/Tile.cpp
    terrainlib/Tiler.cpp
    terrainlib/ctb/GlobalGeodetic.cpp
    terrainlib/ctb/GlobalMercator.cpp
    terrainlib/tntn/BinaryIO.cpp
    terrainlib/tntn/Delaunator.cpp
    terrainlib/tntn/DelaunayMesh.cpp
    terrainlib/tntn/DelaunayTriangle.cpp
    terrainlib/tntn/File.cpp
    terrainlib/tntn/gdal_init.cpp
    terrainlib/tntn/geometrix.cpp
    terrainlib/tntn/logging.cpp
    terrainlib/tntn/Mesh2Raster.cpp
    terrainlib/tntn/Mesh.cpp
    terrainlib/tntn/MeshIO.cpp
    terrainlib/tntn/MeshWriter.cpp
    terrainlib/tntn/OFFReader.cpp
    terrainlib/tntn/Points2Mesh.cpp
    terrainlib/tntn/QuadEdge.cpp
    terrainlib/tntn/QuantizedMeshIO.cpp
    terrainlib/tntn/RasterIO.cpp
    terrainlib/tntn/raster_tools.cpp
    terrainlib/tntn/simple_meshing.cpp
    terrainlib/tntn/SuperTriangle.cpp
    terrainlib/tntn/SurfacePoints.cpp
    terrainlib/tntn/TerraMesh.cpp
    terrainlib/tntn/terra_meshing.cpp
    terrainlib/tntn/TerraUtils.cpp
    terrainlib/tntn/util.cpp
    terrainlib/tntn/ZemlyaMesh.cpp
    terrainlib/tntn/zemlya_meshing.cpp
)

set(ATB_UNITTESTS_HEADERS_AND_SOURCES
    unittests/alpine_raster_format.cpp
    unittests/catch2_helpers.h
    unittests/cesium_terra_format.cpp
    unittests/dataset.cpp
    unittests/dataset_reading.cpp
    unittests/grid.cpp
    unittests/image.cpp
    unittests/layer_json.cpp
    unittests/meta_data_generator.cpp
    unittests/parallel_tile_generator.cpp
    unittests/tiler.cpp
    unittests/main.cpp
    unittests/srs_test.cpp
    unittests/progress_indicator_test.cpp
    unittests/algorithms/primitives.cpp
    unittests/algorithms/raster_triangle_scanline.cpp
    unittests/tntn/Delaunay_tests.cpp
    unittests/tntn/File_tests.cpp
    unittests/tntn/geometrix_tests.cpp
    unittests/tntn/Mesh_tests.cpp
    unittests/tntn/ObjPool_tests.cpp
    unittests/tntn/OFFReader_tests.cpp
    unittests/tntn/QuantizedMeshIO_tests.cpp
    unittests/tntn/RasterIO_tests.cpp
    unittests/tntn/Raster_tests.cpp
    unittests/tntn/raster_tools_tests.cpp
    unittests/tntn/simple_meshing_tests.cpp
    unittests/tntn/SimpleRange_tests.cpp
    unittests/tntn/SuperTriangle_tests.cpp
    unittests/tntn/SurfacePoints_tests.cpp
    unittests/tntn/terra_meshing_tests.cpp
    unittests/tntn/test_common.cpp
    unittests/tntn/test_common.h
    unittests/tntn/triangle_indices.cpp
    unittests/tntn/triangle_indices_data.cpp
    unittests/tntn/triangle_indices.h
    unittests/tntn/util_tests.cpp
    unittests/tntn/vertex_points.cpp
    unittests/tntn/vertex_points.h
)

set(ATB_TERRAINBUILDER_SOURCES
    terrainbuilder/main.cpp
)



if (ATB_ENABLE_ADDRESS_SANITIZER)
    message(NOTICE "building with address sanitizer enabled")
    set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
    set (CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
endif()
if (ATB_ENABLE_THREAD_SANITIZER)
    message(NOTICE "building with thread sanitizer enabled")
    message(WARN ": use the thread sanitizer supression file, e.g.: TSAN_OPTIONS=\"suppressions=thread_sanitizer_suppression.txt\" ./terrainbuilder")
    set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=thread")
    set (CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=thread")
endif()

if (ATB_USE_LLVM_LINKER)
    string(APPEND CMAKE_EXE_LINKER_FLAGS " -fuse-ld=lld")
endif()


add_library(terrainlib STATIC
    ${ATB_TERRAINLIB_HEADERS}
    ${ATB_TERRAINLIB_SOURCES}
)
target_compile_definitions(terrainlib PUBLIC GLM_FORCE_SWIZZLE GLM_ENABLE_EXPERIMENTAL GLM_FORCE_XYZW_ONLY)
target_link_libraries(terrainlib PUBLIC ZLIB::ZLIB GDAL::GDAL fmt freeimage TBB::tbb)
target_include_directories(terrainlib
    PUBLIC terrainlib
    SYSTEM PUBLIC
    ${ATB_INCLUDE_PATHS}
)
if (ATB_ENABLE_ASSERTS)
    target_compile_options(terrainlib PUBLIC "-U NDEBUG")
    target_compile_options(terrainlib PUBLIC "-DTNTN_ASSERT_ACTIVE")
endif()


add_executable(terrainbuilder
    ${ATB_TERRAINBUILDER_SOURCES}
)
target_link_libraries(terrainbuilder PUBLIC terrainlib)

if (ATB_UNITTESTS)
    add_executable(unittests ${ATB_UNITTESTS_HEADERS_AND_SOURCES})
    target_compile_definitions(unittests PUBLIC "ATB_TEST_DATA_DIR=\"${CMAKE_SOURCE_DIR}/unittests/data/\"")
    if (ATB_UNITTESTS_EXTENDED)
        target_compile_definitions(unittests PUBLIC "ATB_UNITTESTS_EXTENDED=true")
    else()
        target_compile_definitions(unittests PUBLIC "ATB_UNITTESTS_EXTENDED=false")
    endif()
    if (ATB_UNITTESTS_DEBUG_IMAGES)
        target_compile_definitions(unittests PUBLIC "ATB_UNITTESTS_DEBUG_IMAGES=true")
    else()
        target_compile_definitions(unittests PUBLIC "ATB_UNITTESTS_DEBUG_IMAGES=false")
    endif()
    target_link_libraries(unittests PUBLIC terrainlib Catch2::Catch2)
endif()
