#include "DatasetReader.h"
#include "Dataset.h"

#include <fmt/core.h>
#include <gdal.h>
#include <memory>
#include <ogr_spatialref.h>
#include <gdal_priv.h>
#include <gdalwarper.h>

#include "Dataset.h"
#include "Exception.h"
#include "Image.h"
#include "ctb/types.hpp"

namespace {
std::string toWkt(const OGRSpatialReference& srs) {
  char *wkt_char_string = nullptr;
  srs.exportToWkt(&wkt_char_string);
  std::string wkt_string(wkt_char_string);
  CPLFree(wkt_char_string);
  return wkt_string;
}

std::array<double, 6> computeGeoTransform(const ctb::CRSBounds& bounds, unsigned width, unsigned height) {
  return {bounds.getMinX(), bounds.getWidth() / width, 0,
          bounds.getMaxY(), 0, -bounds.getHeight() / height};
  }

std::unique_ptr<void, decltype(&GDALDestroyGenImgProjTransformer)> makeImageTransformArgs(const DatasetReader& reader,
                                                                                          Dataset* dataset,
                                                                                          const ctb::CRSBounds& bounds, unsigned width, unsigned height)
{
  CPLStringList transformOptions;
  if (reader.isReprojecting()) {
    transformOptions.SetNameValue("SRC_SRS", reader.dataset_srs_wkt().c_str());
    transformOptions.SetNameValue("DST_SRS", reader.target_srs_wkt().c_str());
  }
  auto args = std::unique_ptr<void, decltype(&GDALDestroyGenImgProjTransformer)>(GDALCreateGenImgProjTransformer2(dataset->gdalDataset(), nullptr, transformOptions.List()), &GDALDestroyGenImgProjTransformer);
  if (!args) {
    throw Exception("GDALCreateGenImgProjTransformer2 failed.");
  }

  const auto adfGeoTransform = computeGeoTransform(bounds, width, height);
  GDALSetGenImgProjTransformerDstGeoTransform(args.get(), adfGeoTransform.data());


  return args;
}

struct MyWarpOptions {
  std::unique_ptr<GDALWarpOptions, decltype (&GDALDestroyWarpOptions)> gdal = {nullptr, &GDALDestroyWarpOptions};
};

MyWarpOptions make_warp_options(const DatasetReader& reader, Dataset* dataset, const ctb::CRSBounds& bounds, unsigned width, unsigned height)
{
  MyWarpOptions options;
  options.gdal.reset(GDALCreateWarpOptions());
  options.gdal->hSrcDS = dataset->gdalDataset();
  options.gdal->nBandCount = 1;
  options.gdal->eResampleAlg = GDALResampleAlg::GRA_Cubic;
  options.gdal->panSrcBands = static_cast<int *>(CPLMalloc(sizeof(int) * 1));
  options.gdal->panDstBands = static_cast<int *>(CPLMalloc(sizeof(int) * 1));
  options.gdal->padfSrcNoDataReal = static_cast<double *>(CPLMalloc(sizeof(double) * 1));
  options.gdal->padfSrcNoDataImag = static_cast<double *>(CPLMalloc(sizeof(double) * 1));
  options.gdal->padfDstNoDataReal = static_cast<double *>(CPLMalloc(sizeof(double) * 1));
  options.gdal->padfDstNoDataImag = static_cast<double *>(CPLMalloc(sizeof(double) * 1));
  {
    int bGotNoData = false;
    double noDataValue = dataset->gdalDataset()->GetRasterBand(1)->GetNoDataValue(&bGotNoData);
    if (!bGotNoData) noDataValue = -32768;

    options.gdal->padfSrcNoDataReal[0] = noDataValue;
    options.gdal->padfSrcNoDataImag[0] = 0;
    options.gdal->padfDstNoDataReal[0] = noDataValue;
    options.gdal->padfDstNoDataImag[0] = 0;

    options.gdal->panSrcBands[0] = int(reader.dataset_band());
    options.gdal->panDstBands[0] = 1;
  }
  options.gdal->pTransformerArg = makeImageTransformArgs(reader, dataset, bounds, width, height).release();
  options.gdal->pfnTransformer = GDALGenImgProjTransform;

  return options;
}

std::shared_ptr<Dataset> getOverviewDataset(const std::shared_ptr<Dataset>& dataset, void *hTransformerArg) {
  GDALDataset* poSrcDS = dataset->gdalDataset();
  int nOvLevel = -2;
  int nOvCount = poSrcDS->GetRasterBand(1)->GetOverviewCount();
  if( nOvCount > 0 )
  {
    double adfSuggestedGeoTransform[6];
    double adfExtent[4];
    int    nPixels, nLines;
    /* Compute what the "natural" output resolution (in pixels) would be for this */
    /* input dataset */
    if( GDALSuggestedWarpOutput2(poSrcDS, GDALGenImgProjTransform, hTransformerArg,
                                 adfSuggestedGeoTransform, &nPixels, &nLines,
                                 adfExtent, 0) == CE_None)
    {
      double dfTargetRatio = 1.0 / adfSuggestedGeoTransform[1];
      if( dfTargetRatio > 1.0 )
      {
        int iOvr;
        for( iOvr = -1; iOvr < nOvCount-1; iOvr++ )
        {
          double dfOvrRatio = (iOvr < 0) ? 1.0 : (double)poSrcDS->GetRasterXSize() /
                                                   poSrcDS->GetRasterBand(1)->GetOverview(iOvr)->GetXSize();
          double dfNextOvrRatio = (double)poSrcDS->GetRasterXSize() /
                                  poSrcDS->GetRasterBand(1)->GetOverview(iOvr+1)->GetXSize();
          if( dfOvrRatio < dfTargetRatio && dfNextOvrRatio > dfTargetRatio )
            break;
          if( fabs(dfOvrRatio - dfTargetRatio) < 1e-1 )
            break;
        }
        iOvr += (nOvLevel+2);
        if( iOvr >= 0 )
        {
          //std::cout << "CTB WARPING: Selecting overview level " << iOvr << " for output dataset " << nPixels << "x" << nLines << std::endl;
          return std::make_shared<Dataset>(static_cast<GDALDataset*>(GDALCreateOverviewDataset( poSrcDS, iOvr, FALSE )));
        }
      }
    }
  }
  return {};
}

}


DatasetReader::DatasetReader(const std::shared_ptr<Dataset>& dataset, const OGRSpatialReference& targetSRS, unsigned band) :
    m_dataset(dataset), m_dataset_srs_wkt(toWkt(dataset->srs())), m_target_srs_wkt(toWkt(targetSRS)), m_requires_reprojection(!dataset->srs().IsSame(&targetSRS)), m_band(band)
{
  if (band > dataset->n_bands())
    throw Exception(fmt::format("Dataset does not contain band number {} (there are {} bands).", band, dataset->n_bands()));
}


HeightData DatasetReader::read(const ctb::CRSBounds& bounds, unsigned width, unsigned height) const
{
  auto warpOptions = make_warp_options(*this, m_dataset.get(), bounds, width, height);
  auto adfGeoTransform = computeGeoTransform(bounds, width, height);
  auto warped_dataset = Dataset(static_cast<GDALDataset *>(GDALCreateWarpedVRT(m_dataset->gdalDataset(), int(width), int(height), adfGeoTransform.data(), warpOptions.gdal.get())));

  auto* heights_band = warped_dataset.gdalDataset()->GetRasterBand(1);  // non-owning pointer
  auto heights_data = HeightData(width, height);
  if (heights_band->RasterIO(GF_Read, 0, 0, int(width), int(height),
                             static_cast<void*>(heights_data.data()), int(width), int(height), GDT_Float32, 0, 0) != CE_None)
    throw Exception("couldn't read data");


  return heights_data;
}

HeightData DatasetReader::readWithOverviews(const ctb::CRSBounds& bounds, unsigned width, unsigned height) const
{
  auto warpOptions = make_warp_options(*this, m_dataset.get(), bounds, width, height);
  auto adfGeoTransform = computeGeoTransform(bounds, width, height);

  const auto overview = getOverviewDataset(m_dataset, warpOptions.gdal->pTransformerArg);
  if (overview) {
    warpOptions.gdal->hSrcDS = overview->gdalDataset();

    // We need to recreate the transform when operating on an overview.
    warpOptions.gdal->pTransformerArg = makeImageTransformArgs(*this, overview.get(), bounds, width, height).release();
  }

  auto warped_dataset = Dataset(static_cast<GDALDataset *>(GDALCreateWarpedVRT(overview ? overview->gdalDataset() : m_dataset->gdalDataset(),
                                                                               int(width), int(height), adfGeoTransform.data(), warpOptions.gdal.get())));

  auto* heights_band = warped_dataset.gdalDataset()->GetRasterBand(1);  // non-owning pointer
  auto heights_data = HeightData(width, height);
  if (heights_band->RasterIO(GF_Read, 0, 0, int(width), int(height),
                             static_cast<void*>(heights_data.data()), int(width), int(height), GDT_Float32, 0, 0) != CE_None)
    throw Exception("couldn't read data");


  return heights_data;
}

