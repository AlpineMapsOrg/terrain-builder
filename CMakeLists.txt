cmake_minimum_required(VERSION 3.18)

project(alpine-terrain-builder)

option(ATB_UNITTESTS "include unit test targets in the buildsystem" ON)
option(ATB_ENABLE_ADDRESS_SANITIZER "compiles ctb with address sanitizer enabled (works only on g++ and clang)" ON)
option(ATB_USE_LLVM_LINKER "use lld (llvm) for linking. it's parallel and much faster, but not installed by default. if it's not installed, you'll get errors, that openmp or other stuff is not installed (hard to track down)" OFF)

set(CMAKE_CXX_STANDARD 20)

find_package(GDAL REQUIRED)
if(NOT GDAL_CONFIG)
    message(FATAL_ERROR "gdal-config command not found (not in PATH?), cannot proceed")
endif()

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
    terrainlib/Dataset.h
)
set(ATB_TERRAINLIB_SOURCES
    terrainlib/Dataset.cpp
)

set(ATB_UNITTESTS_SOURCES
    unittests/main.cpp
)

set(ATB_TERRAINBUILDER_SOURCES
    terrainbuilder/main.cpp
)



if (ATB_ENABLE_ADDRESS_SANITIZER)
    message(NOTICE "building with address sanitizer enabled")
    set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
    set (CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
endif()
if (ATB_USE_LLVM_LINKER)
    string(APPEND CMAKE_EXE_LINKER_FLAGS " -fuse-ld=lld")
endif()



add_library(terrainlib STATIC
    ${ATB_TERRAINLIB_HEADERS}
    ${ATB_TERRAINLIB_SOURCES}
)
target_compile_definitions(terrainlib PUBLIC GLM_FORCE_SWIZZLE GLM_ENABLE_EXPERIMENTAL)
target_link_libraries(terrainlib PUBLIC ZLIB::ZLIB GDAL::GDAL fmt)
target_include_directories(terrainlib
    SYSTEM PUBLIC
    ${ATB_INCLUDE_PATHS}
)


add_executable(terrainbuilder
    ${ATB_TERRAINBUILDER_SOURCES}
)
target_link_libraries(terrainbuilder PUBLIC terrainlib)

if (ATB_UNITTESTS)
    add_executable(unittests
        ${ATB_UNITTESTS_SOURCES}
    )
    target_link_libraries(unittests PUBLIC terrainlib Catch2::Catch2)
endif()
