cmake_minimum_required(VERSION 3.19)
project(terrain_builder_src LANGUAGES CXX)

option(ALP_AUTOUPDATE_RADIX "Keeps whack up-to-date with origin/main, but prevents local edits. Change to OFF if you want to edit radix" ON)
option(ALP_ENABLE_THREAD_SANITIZER "compiles atb with thread sanitizer enabled (only debug, works only on g++ and clang)" OFF)
option(ALP_ENABLE_OVERVIEW_READING "enable GDAL overview reading (broken with newer GDAL versions)" OFF)


find_package(GDAL REQUIRED)
find_package(TBB REQUIRED)
find_package(ZLIB REQUIRED)

if(NOT GDAL_CONFIG)
    message(FATAL_ERROR "gdal-config command not found (not in PATH?), cannot proceed")
endif()
execute_process(
  COMMAND ${GDAL_CONFIG} --version
  OUTPUT_VARIABLE SYSTEM_GDAL_VERSION
)

if(SYSTEM_GDAL_VERSION VERSION_LESS "3.3")
    message(FATAL_ERROR "GDAL version \"${SYSTEM_GDAL_VERSION}\" is too old, at least 3.2 is required")
endif()


include(FetchContent)
if(NOT TARGET fmt::fmt)
    FetchContent_Declare(fmt
        GIT_REPOSITORY https://github.com/fmtlib/fmt.git
        SOURCE_DIR 		${CMAKE_SOURCE_DIR}/3rdparty/fmt
    )
    FetchContent_MakeAvailable(fmt)
endif()
if(ALP_AUTOUPDATE_RADIX)
    FetchContent_Declare(radix
        GIT_REPOSITORY  https://github.com/AlpineMapsOrg/radix.git
        GIT_TAG         origin/main
        SOURCE_DIR      ${CMAKE_SOURCE_DIR}/3rdparty/radix
        SOURCE_SUBDIR   src
    )
    FetchContent_MakeAvailable(radix)
else()
    if (NOT EXISTS ${CMAKE_SOURCE_DIR}/3rdparty/radix)
        FetchContent_Declare(radix
            GIT_REPOSITORY  https://github.com/AlpineMapsOrg/radix.git
            GIT_TAG         main
            SOURCE_DIR      ${CMAKE_SOURCE_DIR}/3rdparty/radix
        )
        FetchContent_MakeAvailable(radix)
    else()
        add_subdirectory(${CMAKE_SOURCE_DIR}/3rdparty/radix ${CMAKE_BINARY_DIR}/_deps/radix-build)
    endif()
endif()

if(NOT TARGET curl)
    find_package(CURL REQUIRED)
endif()

add_library(terrainlib
    terrainlib/alpine_raster.h terrainlib/alpine_raster.cpp

    terrainlib/Dataset.h terrainlib/Dataset.cpp
    terrainlib/DatasetReader.h terrainlib/DatasetReader.cpp
    terrainlib/depth_first_tile_traverser.h
    terrainlib/Exception.h
    terrainlib/Image.h terrainlib/Image.cpp
    terrainlib/ParallelTileGenerator.h terrainlib/ParallelTileGenerator.cpp
    terrainlib/ParallelTiler.h terrainlib/ParallelTiler.cpp
    terrainlib/ProgressIndicator.h terrainlib/ProgressIndicator.cpp
    terrainlib/srs.h
    terrainlib/TileHeightsGenerator.h terrainlib/TileHeightsGenerator.cpp
    terrainlib/Tiler.h terrainlib/Tiler.cpp
    terrainlib/TopDownTiler.h terrainlib/TopDownTiler.cpp
    terrainlib/algorithms/primitives.h
    terrainlib/algorithms/raster_triangle_scanline.h
    terrainlib/ctb/CTBException.hpp
    terrainlib/ctb/GlobalGeodetic.hpp terrainlib/ctb/GlobalGeodetic.cpp
    terrainlib/ctb/GlobalMercator.hpp terrainlib/ctb/GlobalMercator.cpp
    terrainlib/ctb/Grid.hpp
    terrainlib/ctb/TileCoordinate.hpp
    terrainlib/ctb/types.hpp
    terrainlib/tntn/BinaryIO.h terrainlib/tntn/BinaryIO.cpp
    terrainlib/tntn/Delaunator.h terrainlib/tntn/Delaunator.cpp
    terrainlib/tntn/DelaunayMesh.h terrainlib/tntn/DelaunayMesh.cpp
    terrainlib/tntn/DelaunayTriangle.h terrainlib/tntn/DelaunayTriangle.cpp
    terrainlib/tntn/endianness.h
    terrainlib/tntn/FileFormat.h
    terrainlib/tntn/File.h terrainlib/tntn/File.cpp
    terrainlib/tntn/gdal_init.h terrainlib/tntn/gdal_init.cpp
    terrainlib/tntn/geometrix.h terrainlib/tntn/geometrix.cpp
    terrainlib/tntn/logging.h terrainlib/tntn/logging.cpp
    terrainlib/tntn/Mesh2Raster.h terrainlib/tntn/Mesh2Raster.cpp
    terrainlib/tntn/Mesh.h terrainlib/tntn/Mesh.cpp
    terrainlib/tntn/MeshIO.h terrainlib/tntn/MeshIO.cpp
    terrainlib/tntn/MeshMode.h
    terrainlib/tntn/MeshWriter.h terrainlib/tntn/MeshWriter.cpp
    terrainlib/tntn/ObjPool.h
    terrainlib/tntn/OFFReader.h terrainlib/tntn/OFFReader.cpp
    terrainlib/tntn/Points2Mesh.h terrainlib/tntn/Points2Mesh.cpp
    terrainlib/tntn/QuadEdge.h terrainlib/tntn/QuadEdge.cpp
    terrainlib/tntn/QuantizedMeshIO.h terrainlib/tntn/QuantizedMeshIO.cpp
    terrainlib/tntn/Raster.h
    terrainlib/tntn/RasterIO.h terrainlib/tntn/RasterIO.cpp
    terrainlib/tntn/raster_tools.h terrainlib/tntn/raster_tools.cpp
    terrainlib/tntn/simple_meshing.h terrainlib/tntn/simple_meshing.cpp
    terrainlib/tntn/SuperTriangle.h terrainlib/tntn/SuperTriangle.cpp
    terrainlib/tntn/SurfacePoints.h terrainlib/tntn/SurfacePoints.cpp
    terrainlib/tntn/TerraMesh.h terrainlib/tntn/TerraMesh.cpp
    terrainlib/tntn/terra_meshing.h terrainlib/tntn/terra_meshing.cpp
    terrainlib/tntn/TerraUtils.h terrainlib/tntn/TerraUtils.cpp
    terrainlib/tntn/tntn_assert.h
    terrainlib/tntn/util.h terrainlib/tntn/util.cpp
    terrainlib/tntn/ZemlyaMesh.h terrainlib/tntn/ZemlyaMesh.cpp
    terrainlib/tntn/zemlya_meshing.h terrainlib/tntn/zemlya_meshing.cpp
    terrainlib/tntn/ZoomRange.h
)
target_link_libraries(terrainlib PUBLIC radix ZLIB::ZLIB GDAL::GDAL fmt freeimage TBB::tbb)
target_include_directories(terrainlib PUBLIC ./terrainlib)
if (ALP_ENABLE_OVERVIEW_READING)
    target_compile_definitions(terrainlib PUBLIC DATB_ENABLE_OVERVIEW_READING)
endif()


add_executable(terrainbuilder
    terrainbuilder/main.cpp
)
target_link_libraries(terrainbuilder PUBLIC terrainlib)


add_executable(tile-downloader
    tile_downloader/main.cpp
)
target_link_libraries(tile-downloader PUBLIC terrainlib ${CURL_LIBRARIES})
target_include_directories(tile-downloader PUBLIC ${CURL_INCLUDE_DIR})
