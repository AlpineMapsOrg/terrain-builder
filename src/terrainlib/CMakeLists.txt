# *****************************************************************************
# AlpineMaps.org
# Copyright (C) 2026 Adam Celarek-Litofcenko
# Copyright (C) 2026 Martin Braunsperger
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

add_library(terrainlib
    Error.h Error.cpp
    ImageKey.h
    PrunedLocation.h PrunedLocation.cpp
    Range.h
    SphereProjector.h
    VecHash.h
    VecRange.h
    WorkerGroup.h
    array_utils.h
    build_config.h
    common_error_codes.h
    earth.h
    enumerate.h
    fmt_impls.h
    glm_utils.h
    hash_utils.h
    macros.h
    mem.h
    opencv_utils.h
    optional_utils.h
    parallel.h
    path_utils.h
    pch.h
    range_utils.h
    span_utils.h
    stop.h
    string_utils.h
    tile_path.h
    type_name.h
    type_utils.h
    variant_utils.h
    vector_utils.h

    cli_support/ProgressIndicator.h cli_support/ProgressIndicator.cpp
    cli_support/log.h cli_support/log.cpp
    cli_support/stop_signal.h cli_support/stop_signal.cpp
    cli_support/octree_id.h

    containers/Buffer.h
    containers/ChunkedRingBuffer.h
    containers/ChunkedRingWriter.h
    containers/Cow.h
    containers/FixedArray.h
    containers/FixedString.h
    containers/FixedVector.h
    containers/HybridIndexPairMap.h
    containers/HybridVector.h
    containers/OffsetTable.h
    containers/OffsetVector.h
    containers/OnceCell.h
    containers/SegmentedBuffer.h
    containers/Size.h
    containers/StampSet.h
    containers/TinyVector.h
    containers/UnionFind.h
    containers/UniqueResource.h
    containers/Vector2D.h
    containers/indirect.h
    containers/spsc.h

    error/CodeStorage.h
    error/Context.h
    error/ErasedCode.h
    error/Error.h
    error/ErrorCode.h
    error/FallibleFn.h
    error/Infallible.h
    error/MessageCode.h
    error/Result.h
    error/ResultFn.h
    error/Trace.h
    error/TypedCode.h
    error/format.h
    error/log.h
    error/prelude.h
    error/try.h
    error/unwrap.h
    error/utils.h

    geometry/CoordFrame.h
    geometry/PlaneFrame.h
    geometry/Rect.h
    geometry/geometry.h

    gis/CplError.h
    gis/Dataset.h gis/Dataset.cpp
    gis/DatasetReader.h gis/DatasetReader.cpp
    gis/GdalError.h
    gis/GeoTransform.h gis/GeoTransform.cpp
    gis/OgrError.h
    gis/PixelTransform.h gis/PixelTransform.cpp
    gis/Transform.h
    gis/gdal.h gis/gdal.cpp
    gis/georeferencing.h
    gis/srs/Axis.h
    gis/srs/Ellipsoid.h
    gis/srs/Kind.h
    gis/srs/System.h gis/srs/System.cpp
    gis/srs/ThreadLocalSystem.h
    gis/srs/Transform.h
    gis/srs/Unit.h
    gis/srs/classify.h gis/srs/classify.cpp
    gis/srs/coverage.h gis/srs/coverage.cpp
    gis/srs/detail.h gis/srs/detail.cpp
    gis/srs/error.h
    gis/srs/well_known.h
    gis/vector_mask.h gis/vector_mask.cpp
    gis/webmercator.h gis/webmercator.cpp

    hierarchy/Index.h
    hierarchy/IndexMap.h
    hierarchy/NodeStatus.h
    hierarchy/NodeStatusOrMissing.h
    hierarchy/Traits.h
    hierarchy/traverse.h

    io/BufferLimitExceeded.h
    io/FileError.h
    io/OpenCvError.h
    io/PayloadSchema.h
    io/UnknownExtension.h
    io/WriteMode.h
    io/ZppBitsError.h
    io/ZstdError.h
    io/alloc.h
    io/bytes.h io/bytes.cpp
    io/compression.h io/compression.cpp
    io/conversion.h
    io/envelope.h io/envelope.cpp
    io/filesystem.h io/filesystem.cpp
    io/glm_serialization.h
    io/hash.h io/hash.cpp
    io/image.h io/image.cpp
    io/image/Format.h
    io/image/JxlError.h
    io/json.h
    io/json/Value.h
    io/json/file.h io/json/file.cpp
    io/json/serde.h io/json/serde.cpp
    io/limits.h
    io/mat.h io/mat.cpp
    io/radix_serialization.h
    io/serde.h
    io/text.h

    mesh/EncodedMesh.h
    mesh/PointWithMeta.h
    mesh/SimpleMesh.h
    mesh/TriangleContainer.h
    mesh/TriangleSoup.h
    mesh/VertexMap.h
    mesh/View.h
    mesh/WindingOrder.h
    mesh/bfs.h mesh/bfs.cpp
    mesh/boolean.h mesh/boolean.cpp
    mesh/bounds.h
    mesh/cgal.h
    mesh/cleanup.h mesh/cleanup.cpp
    mesh/clip.h mesh/clip.cpp
    mesh/codec/Gltf.h
    mesh/codec/SfMesh.h
    mesh/codec/resolve.h mesh/codec/resolve.cpp
    mesh/combine.h
    mesh/connectivity/adjacency.h mesh/connectivity/adjacency.cpp
    mesh/connectivity/boundary.h mesh/connectivity/boundary.cpp
    mesh/connectivity/connected_components.h
    mesh/connectivity/edges.h
    mesh/connectivity/manifold.h mesh/connectivity/manifold.cpp
    mesh/connectivity/topology.h mesh/connectivity/topology.cpp
    mesh/connectivity/triangle_compare.h
    mesh/connectivity/vertex_index_range.h mesh/connectivity/vertex_index_range.cpp
    mesh/convert.h mesh/convert.cpp
    mesh/encode.h
    mesh/encode_error.h
    mesh/geometry.h mesh/geometry.cpp
    mesh/holes.h mesh/holes.cpp
    mesh/igl/convert.h
    mesh/igl/cut.h
    mesh/igl/cut_to_disk.h
    mesh/igl/igl.cpp
    mesh/igl/manifold.h
    mesh/igl/orient.h
    mesh/io.h mesh/io.cpp
    mesh/io/CgltfError.h
    mesh/io/gltf.h mesh/io/gltf.cpp
    mesh/io/options.h
    mesh/io/sf_mesh.h mesh/io/sf_mesh.cpp
    mesh/merge.h
    mesh/merging/EpsilonVertexDeduplicate.h
    mesh/merging/ExactVertexDeduplicate.h
    mesh/merging/SphereProjectionVertexDeduplicate.h
    mesh/merging/VertexDeduplicate.h
    mesh/merging/VertexId.h
    mesh/merging/VertexMapping.h
    mesh/merging/helpers.h
    mesh/merging/mapping.h mesh/merging/mapping.cpp
    mesh/non_manifold_result.h
    mesh/normalize.h
    mesh/reindex.h
    mesh/split.h
    mesh/storage.h
    mesh/texture_trim.h mesh/texture_trim.cpp
    mesh/utils.h
    mesh/validate/assert.h mesh/validate/debug_assert.h
    mesh/validate/cgal.h
    mesh/validate/flags.h mesh/validate/result.h mesh/validate/result.cpp

    meshopt/encode.h

    numeric/StrongDouble.h
    numeric/checked.h
    numeric/glm_math.h
    numeric/int_math.h
    numeric/number_types.h
    numeric/number_utils.h
    numeric/overflow.h
    numeric/quantize.h
    numeric/saturating.h
    numeric/wide_integer.h

    octree/Id.h
    octree/IdRect.h
    octree/OddLevelShifted.h octree/OddLevelShifted.cpp
    octree/Space.h octree/Space.cpp
    octree/StoreTraits.h
    octree/id_serialization.h
    octree/path_layout.h
    octree/path_layout/flat.h
    octree/path_layout/level_and_coordinate_directories.h
    octree/storage/Index.h
    octree/storage/create.h
    octree/storage/index_format.h octree/storage/index_format.cpp
    octree/storage/metadata.h octree/storage/metadata.cpp
    octree/storage/open.h
    octree/storage/publish.h
    octree/utils.h

    polygon/Polygon.h
    polygon/clip.h polygon/clip.cpp
    polygon/triangulate.h polygon/triangulate.cpp
    polygon/utils.h polygon/utils.cpp

    range/AnyRange.h
    range/Bound.h
    range/Range.h
    range/RangeBounds.h
    range/RangeFrom.h
    range/RangeFull.h
    range/RangeInclusive.h
    range/RangeTo.h
    range/RangeToInclusive.h

    raster/ClampedView.h
    raster/View.h
    raster/algorithm.h
    raster/algorithm/conversion.h
    raster/algorithm/copy.h
    raster/algorithm/detail.h
    raster/algorithm/fold.h
    raster/algorithm/reduce.h
    raster/algorithm/reducers.h
    raster/algorithm/scale.h
    raster/algorithm/scaling_detail.h
    raster/algorithm/transform.h
    raster/algorithm/window_transform.h
    raster/algorithm/zip_transform.h

    raster_store/StoreTraits.h
    raster_store/Tile.h
    raster_store/TileCodec.h
    raster_store/TilePool.h
    raster_store/attribution/Table.h
    raster_store/attribution/table_file.h raster_store/attribution/table_file.cpp
    raster_store/path_layout.h
    raster_store/path_layout/zoom_x_y_google.h
    raster_store/pixel.h
    raster_store/scaler.h
    raster_store/storage.h
    raster_store/storage/Index.h
    raster_store/storage/index_format.h raster_store/storage/index_format.cpp
    raster_store/storage/metadata.h raster_store/storage/metadata.cpp

    sf/open.h
    sf/publish.h
    sf/validate_index.h

    spatial_lookup/CellBased.h
    spatial_lookup/CellBasedStorage.h
    spatial_lookup/Grid.h
    spatial_lookup/GridStorage.h
    spatial_lookup/Hashmap.h
    spatial_lookup/HashmapStorage.h
    spatial_lookup/NDLoopHelper.h
    spatial_lookup/QuantizedHash.h
    spatial_lookup/SpatialLookup.h

    store/Checkpoint.h
    store/Checkpointer.h
    store/Codec.h
    store/Draft.h
    store/Hash.h
    store/Index.h
    store/IndexFormat.h
    store/KeyTraits.h
    store/Metadata.h
    store/PersistedIndex.h
    store/RawStorage.h
    store/SimpleIndex.h
    store/Storage.h
    store/View.h
    store/create.h
    store/envelope_utils.h
    store/fingerprint.h
    store/index_checks.h
    store/inputs.h store/inputs.cpp
    store/open.h
    store/path_layout.h
    store/publish.h store/publish.cpp
    store/storage_paths.h store/storage_paths.cpp

    texture/Mask.h
    texture/dilate_colors.h texture/dilate_colors.cpp

    uv/atlas.h uv/atlas.cpp
    uv/fit.h
    uv/unwrap.h uv/unwrap.cpp
)
set_source_files_properties(
    mesh/igl/igl.cpp
    mesh/io/gltf.cpp
    PROPERTIES SKIP_UNITY_BUILD_INCLUSION ON
)

target_compile_features(terrainlib PUBLIC cxx_std_23)
target_compile_definitions(terrainlib PUBLIC SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)
target_include_directories(terrainlib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_precompile_headers(terrainlib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/pch.h)
if(ALP_WARNINGS_AS_ERRORS)
    target_compile_options(terrainlib PUBLIC
        $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang,AppleClang>:-Werror>
        $<$<COMPILE_LANG_AND_ID:CXX,MSVC>:/WX>
    )
endif()
target_link_libraries(terrainlib PUBLIC
    radix
    ZLIB::ZLIB
    CGAL::CGAL
    Eigen3
    GDAL::GDAL
    spdlog
    fmt
    zpp_bits
    value_types
    zstd::libzstd_static
    cgltf
    TBB::tbb
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    libassert::assert
    magic_enum::magic_enum
    meshoptimizer
    WideInteger
    igl::core
    igl_restricted::triangle
    xatlas
)

# Error captures stack traces; libassert already links cpptrace publicly.
target_link_libraries(terrainlib PRIVATE cpptrace::cpptrace)
# Only io/hash.cpp includes xxHash.
target_link_libraries(terrainlib PRIVATE xxhash)
# Only io/image.cpp and io/image/JxlError.h include libjxl.
target_link_libraries(terrainlib PRIVATE JpegXL::jxl)
