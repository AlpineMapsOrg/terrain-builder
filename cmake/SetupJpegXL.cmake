#############################################################################
# AlpineMaps.org
# Copyright (C) 2026 Adam Celarek <family name at cg tuwien ac at>
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
#############################################################################

if(NOT COMMAND alp_setup_cmake_project)
    include(${CMAKE_CURRENT_LIST_DIR}/SetupCMakeProject.cmake)
endif()

# libjxl installs only pkg-config files. JpegXL::jxl is therefore defined from
# the static libraries of the install, which also contains highway and brotli.
function(alp_setup_jpegxl version)
    alp_setup_cmake_project(libjxl
        URL https://github.com/libjxl/libjxl.git COMMITISH ${version}
        CMAKE_ARGUMENTS
            -DBUILD_SHARED_LIBS=OFF
            -DBUILD_TESTING=OFF
            -DJPEGXL_ENABLE_TOOLS=OFF
            -DJPEGXL_ENABLE_FUZZERS=OFF
            -DJPEGXL_ENABLE_DOXYGEN=OFF
            -DJPEGXL_ENABLE_MANPAGES=OFF
            -DJPEGXL_ENABLE_BENCHMARK=OFF
            -DJPEGXL_ENABLE_EXAMPLES=OFF
            -DJPEGXL_ENABLE_JNI=OFF
            -DJPEGXL_ENABLE_SJPEG=OFF
            -DJPEGXL_ENABLE_OPENEXR=OFF
            -DJPEGXL_ENABLE_TCMALLOC=OFF
            -DJPEGXL_BUNDLE_LIBPNG=OFF
        )

    find_path(_alp_jpegxl_include_dir jxl/decode.h
        PATHS "${ALP_libjxl_INSTALL_DIR}/include" NO_DEFAULT_PATH NO_CACHE REQUIRED)
    # Static link order: dependents before their dependencies.
    set(_alp_jpegxl_libraries)
    foreach(_alp_name IN ITEMS jxl jxl_cms hwy brotlidec brotlienc brotlicommon)
        find_library(_alp_jpegxl_${_alp_name} NAMES ${_alp_name}
            PATHS "${ALP_libjxl_INSTALL_DIR}" PATH_SUFFIXES lib lib64 NO_DEFAULT_PATH NO_CACHE REQUIRED)
        list(APPEND _alp_jpegxl_libraries "${_alp_jpegxl_${_alp_name}}")
    endforeach()

    # Global, because static libraries pass their private link dependencies on to every linking directory.
    add_library(JpegXL::jxl INTERFACE IMPORTED GLOBAL)
    target_include_directories(JpegXL::jxl INTERFACE "${_alp_jpegxl_include_dir}")
    target_compile_definitions(JpegXL::jxl INTERFACE JXL_STATIC_DEFINE JXL_CMS_STATIC_DEFINE)
    target_link_libraries(JpegXL::jxl INTERFACE ${_alp_jpegxl_libraries})
endfunction()
