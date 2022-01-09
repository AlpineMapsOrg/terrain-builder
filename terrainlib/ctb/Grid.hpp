#ifndef CTBGRID_HPP
#define CTBGRID_HPP

/*******************************************************************************
 * Copyright 2014 GeoData <geodata@soton.ac.uk>
 * Copyright 2022 Adam Celarek <family name at cg tuwien ac at>
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License.  You may obtain a copy
 * of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *******************************************************************************/

/**
 * @file Grid.hpp
 * @brief This defines and declares the `Grid` class
 */

#include <cmath>

#include "ogr_spatialref.h"

#include "types.hpp"
#include "TileCoordinate.hpp"

namespace ctb {
  class Grid;
}

/**
 * @brief A generic grid for cutting tile sets
 *
 * This class models a grid for use in cutting up an area into zoom levels and
 * tiles.  It provides functionality such as relating a coordinate in a native
 * coordinate reference system (CRS) to a tile (see `Grid::crsToTile`) and
 * getting the CRS bounds of a tile (see `Grid::tileBounds`).
 *
 * The `Grid` class should be able to model most grid systems. The
 * `GlobalMercator` and `GlobalGeodetic` subclasses implement the specific Tile
 * Mapping Service grid profiles.
 *
 * The code here generalises the logic in the `gdal2tiles.py` script available
 * with the GDAL library.
 *
 * Warning: The y directino is dangerous. Sometimes the positve y axis points north, sometimes not.
 *          - GlobalMercator has y positive pointing north
 *          - GlobalGeodetic as well.
 *          - https://www.maptiler.com/google-maps-coordinates-tile-bounds-projection/#1/175.75/56.27
 *            - google webmercator has tile coordinates where y=0 is the northern most tile.
 *            - tms webmercator has tile coordinates with y=0 being southern most.
 *
 *         Effectively, ctb::Grid is always positive pointing north. Support for google webmercator /
 *         slippyMap is done in Tiler.h
 */
class ctb::Grid {
public:
  enum class Srs {
    SphericalMercator = 3857,
    WGS84 = 4326
  };

  /// Initialise a grid tile
  Grid(i_tile gridSize,
       const CRSBounds extent,
       const OGRSpatialReference& srs,
       int epsgCode,
       unsigned short int rootTiles,
       double zoomFactor):
      mGridSize(gridSize),
      mExtent(extent),
      mSRS(srs),
      mEpsgCode(epsgCode),
      mInitialResolution((extent.getWidth() / rootTiles) / gridSize ),
      mXOriginShift(extent.getWidth() / 2),
      mYOriginShift(extent.getHeight() / 2),
      mZoomFactor(zoomFactor)
  {
    mSRS.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
  }

  /// Override the equality operator
  bool operator==(const Grid &other) const {
    return mGridSize == other.mGridSize
      && mExtent == other.mExtent
      && mSRS.IsSame(&(other.mSRS))
      && mInitialResolution == other.mInitialResolution
      && mXOriginShift == other.mXOriginShift
      && mYOriginShift == other.mYOriginShift
      && mZoomFactor == other.mZoomFactor;
  }

  /// Get the resolution for a particular zoom level
  [[nodiscard]] inline double resolution(i_zoom zoom) const {
    return mInitialResolution / pow(mZoomFactor, zoom);
  }

  /**
   * @brief Get the zoom level for a particular resolution
   *
   * If the resolution does not exactly match a zoom level then the zoom level
   * is 'rounded up' to the next level.
   */
  [[nodiscard]] inline i_zoom zoomForResolution(double resolution) const {
    // if mZoomFactor == 2 the following is the same as using:
    // log2(mInitialResolution) - log2(resolution)
    return i_zoom(ceil((std::log(mInitialResolution)/std::log(mZoomFactor)) - (std::log(resolution)/std::log(mZoomFactor))));
  }

  /// Get the tile covering a pixel location
  [[nodiscard]] inline TilePoint pixelsToTile(const PixelPoint &pixel) const {
    const auto tx = i_tile (pixel.x / mGridSize);
    const auto ty = i_tile (pixel.y / mGridSize);

    return {tx, ty};
  }

  /// Convert pixel coordinates at a given zoom level to CRS coordinates
  [[nodiscard]] inline CRSPoint pixelsToCrs(const PixelPoint &pixel, i_zoom zoom) const {
    double res = resolution(zoom);

    return {
      (pixel.x * res) - mXOriginShift,
      (pixel.y * res) - mYOriginShift
    };
  }

  /// Get the pixel location represented by a CRS point and zoom level
  [[nodiscard]] inline PixelPoint crsToPixels(const CRSPoint &coord, i_zoom zoom) const {
    const auto res = resolution(zoom);
    const auto px = (mXOriginShift + coord.x) / res;
    const auto py = (mYOriginShift + coord.y) / res;

    return {px, py};
  }

  /// Get the tile coordinate in which a location falls at a specific zoom level
  [[nodiscard]] inline TileCoordinate crsToTile(const CRSPoint &coord, i_zoom zoom) const {
    const PixelPoint pixel = crsToPixels(coord, zoom);
    TilePoint tile = pixelsToTile(pixel);

    return {zoom, tile};
  }

  /// Get the CRS bounds of a particular tile
  /// border_se should be true if a border should be included on the south eastern corner
  /// e.g., for the cesium raster terrain format (https://github.com/CesiumGS/cesium/wiki/heightmap-1%2E0)
  [[nodiscard]] inline CRSBounds srsBounds(const TileCoordinate &coord, bool border_se) const {
    // get the pixels coordinates representing the tile bounds
    const PixelPoint pxMinLeft(coord.x * mGridSize, coord.y * mGridSize);
    const PixelPoint pxMaxRight((coord.x + 1) * mGridSize + border_se, (coord.y + 1) * mGridSize + border_se);

    // convert pixels to native coordinates
    const CRSPoint minLeft = pixelsToCrs(pxMinLeft, coord.zoom);
    const CRSPoint maxRight = pixelsToCrs(pxMaxRight, coord.zoom);

    return {minLeft, maxRight};
  }

  /// Get the tile size associated with this grid
  [[nodiscard]] inline i_tile tileSize() const {
    return mGridSize;
  }

  /// Get the tile size associated with this grid
  [[nodiscard]] inline const OGRSpatialReference& getSRS() const {
    return mSRS;
  }

  /// Get the extent covered by the grid in CRS coordinates
  [[nodiscard]] inline const CRSBounds& getExtent() const {
    return mExtent;
  }

  /// Get the extent covered by the grid in tile coordinates for a zoom level
  [[nodiscard]] inline TileBounds getTileExtent(i_zoom zoom) const {
    TileCoordinate ll = crsToTile(mExtent.getLowerLeft(), zoom);
    TileCoordinate ur = crsToTile(mExtent.getUpperRight(), zoom);

    return {ll, ur};
  }

  [[nodiscard]] inline int getEpsgCode() const {
    return mEpsgCode;
  }

private:

  /// The tile size associated with this grid
  i_tile mGridSize;

  /// The area covered by the grid
  CRSBounds mExtent;

  /// The spatial reference system covered by the grid
  OGRSpatialReference mSRS;
  int mEpsgCode = -1;

  double mInitialResolution; ///< The initial resolution of this particular profile
  double mXOriginShift; ///< The shift in CRS coordinates to get to the origin from minx
  double mYOriginShift; ///< The shift in CRS coordinates to get to the origin from miny

  /// By what factor will the scale increase at each zoom level?
  double mZoomFactor;
};

#endif /* CTBGRID_HPP */
