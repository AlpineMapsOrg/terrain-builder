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

# Static libjpeg-turbo: TurboJPEG for terrainlib, and libjpeg for OpenCV through
# ALP_LIBJPEG_TURBO_INCLUDE_DIR, ALP_LIBJPEG_TURBO_JPEG_LIBRARY and ALP_LIBJPEG_TURBO_SIGNATURE.
# Both copies stay private, because the system libjpeg is also loaded (through libtiff).
function(alp_setup_libjpeg_turbo version sha256)
    alp_setup_cmake_project(libjpeg-turbo
        URL https://github.com/libjpeg-turbo/libjpeg-turbo/releases/download/${version}/libjpeg-turbo-${version}.tar.gz
        SHA256 ${sha256}
        CMAKE_ARGUMENTS
            -DENABLE_SHARED=OFF
            -DENABLE_STATIC=ON
            -DWITH_TURBOJPEG=ON
            -DWITH_TOOLS=OFF
            -DWITH_TESTS=OFF
            -DREQUIRE_SIMD=ON
            -DCMAKE_POSITION_INDEPENDENT_CODE=ON
        )

    # Global, because static libraries pass their private link dependencies on to every linking directory.
    find_package(libjpeg-turbo CONFIG REQUIRED GLOBAL)
    # Executables must not export jpeg_*, otherwise the system libjpeg's users would bind to our copy.
    set_property(TARGET libjpeg-turbo::turbojpeg-static APPEND PROPERTY
        INTERFACE_LINK_OPTIONS "LINKER:--exclude-libs,libturbojpeg.a")

    find_path(_alp_libjpeg_turbo_include_dir jpeglib.h
        PATHS "${ALP_libjpeg-turbo_INSTALL_DIR}/include" NO_DEFAULT_PATH NO_CACHE REQUIRED)
    find_library(_alp_libjpeg_turbo_jpeg_library NAMES libjpeg.a
        PATHS "${ALP_libjpeg-turbo_INSTALL_DIR}" PATH_SUFFIXES lib lib64 NO_DEFAULT_PATH NO_CACHE REQUIRED)
    set(ALP_LIBJPEG_TURBO_INCLUDE_DIR "${_alp_libjpeg_turbo_include_dir}" PARENT_SCOPE)
    set(ALP_LIBJPEG_TURBO_JPEG_LIBRARY "${_alp_libjpeg_turbo_jpeg_library}" PARENT_SCOPE)
    # The install's cache key: changes with the version and the build settings.
    set(ALP_LIBJPEG_TURBO_SIGNATURE "$CACHE{ALP_INSTALLED_libjpeg-turbo_VERSION}" PARENT_SCOPE)
endfunction()
