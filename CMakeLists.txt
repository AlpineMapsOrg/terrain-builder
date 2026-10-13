cmake_minimum_required(VERSION 3.25)

project(alpine-terrain-builder LANGUAGES C CXX)

# Executable targets
option(ALP_BUILD_SF_MERGER "Build sf-merger" ${PROJECT_IS_TOP_LEVEL})
option(ALP_BUILD_SF_BUILDER "Build sf-builder" ${PROJECT_IS_TOP_LEVEL})
option(ALP_BUILD_TILE_DOWNLOADER "Build tile-downloader" ${PROJECT_IS_TOP_LEVEL})
option(ALP_BUILD_MESH_CONVERT "Build mesh-convert" ${PROJECT_IS_TOP_LEVEL})
option(ALP_BUILD_SF_INDEX_BROWSER "Build sf-index-browser" ${PROJECT_IS_TOP_LEVEL})
option(ALP_BUILD_DAG_BUILDER "Build dag-builder" ${PROJECT_IS_TOP_LEVEL})
option(ALP_BUILD_DAG_CONVERT_DEBUG "Build dag-convert-debug" ${PROJECT_IS_TOP_LEVEL})
option(ALP_BUILD_SF_BROWSER "Build sf-browser" ${PROJECT_IS_TOP_LEVEL})
option(ALP_BUILD_UNITTESTS "Build unit tests for enabled components" ${PROJECT_IS_TOP_LEVEL})

include(cmake/ConfigureBuild.cmake)

include(cmake/AddRepo.cmake)
include(cmake/SetupCMakeProject.cmake)

add_subdirectory(src)
if(ALP_BUILD_UNITTESTS)
    add_subdirectory(unittests)
endif()
