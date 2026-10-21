#include "srs/System.h"

#include <memory>
#include <string_view>
#include <utility>

#include <cpl_conv.h>
#include <fmt/format.h>
#include <libassert/assert.hpp>

#include "error/Context.h"
#include "error/try.h"
#include "error/utils.h"
#include "string_utils.h"

namespace srs {

namespace {

// The EPSG code of the part of `srs` at `wkt1_path`, such as "DATUM" or "COMPD_CS|VERT_CS", or of `srs` itself if the path is null.
std::optional<uint32_t> get_epsg_code(const OGRSpatialReference &srs, const char *wkt1_path) {
    if (to_string_view(srs.GetAuthorityName(wkt1_path)) != "EPSG") {
        return std::nullopt;
    }

    return to_string_view(srs.GetAuthorityCode(wkt1_path)).and_then([](const std::string_view text) { return from_chars<uint32_t>(text); });
}

}

alp::Result<std::string, gdal::OgrError> System::to_wkt() const {
    const char *const options[] = {"FORMAT=WKT2", nullptr};
    char *raw_wkt = nullptr;
    const OGRErr error = this->_srs.exportToWkt(&raw_wkt, options);
    const std::unique_ptr<char, decltype(&CPLFree)> wkt(raw_wkt, &CPLFree);

    ALP_TRY(gdal::to_result(error));
    return std::string(wkt.get());
}

alp::Result<System, UnsupportedSystem> from_gdal(OGRSpatialReference srs) {
    DEBUG_ASSERT(!srs.IsEmpty());

    // A compound system is geographic or projected if its horizontal part is.
    if (!srs.IsGeographic() && !srs.IsProjected() && !srs.IsGeocentric()) {
        return alp::make_unexpected(UnsupportedSystem::NotGeodetic);
    }

    const std::vector<int> data_axes = srs.GetDataAxisToSRSAxisMapping();
    srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
    if (srs.GetDataAxisToSRSAxisMapping() != data_axes) {
        return alp::make_unexpected(UnsupportedSystem::AxisOrder);
    }

    return System(std::move(srs));
}

alp::Result<System, gdal::OgrError, UnsupportedSystem> from_epsg(const uint32_t code) {
    const auto context = alp::make_context("importing a spatial reference from EPSG code {}", code);

    OGRSpatialReference srs;
    ALP_TRY_CONTEXT(gdal::to_result(srs.importFromEPSG(code)), context);
    srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    ALP_TRY_CONTEXT(System system, from_gdal(std::move(srs)), context);
    return system;
}

alp::Result<System, gdal::OgrError, UnsupportedSystem> from_user_input(const std::string &input) {
    const auto context = alp::make_context("parsing a spatial reference from {}", input);

    OGRSpatialReference srs;
    ALP_TRY_CONTEXT(gdal::to_result(srs.SetFromUserInput(input.c_str())), context);
    srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    ALP_TRY_CONTEXT(System system, from_gdal(std::move(srs)), context);
    return system;
}

std::optional<std::string> format_epsg_code(const System &srs) {
    const OGRSpatialReference &gdal_srs = srs.as_gdal();
    if (const std::optional<uint32_t> code_opt = get_epsg_code(gdal_srs, nullptr)) {
        return fmt::format("EPSG:{}", *code_opt);
    }

    OGRSpatialReference identified = gdal_srs;
    identified.AutoIdentifyEPSG();
    if (const std::optional<uint32_t> code_opt = get_epsg_code(identified, nullptr)) {
        return fmt::format("EPSG:{}", *code_opt);
    }

    if (gdal_srs.IsCompound()) {
        // The search for a node goes depth first, so GEOGCS would find the base of a projected horizontal part. // TODO: use ALP_TRY
        const char *horizontal_node = gdal_srs.IsProjected() ? "PROJCS" : "GEOGCS";
        const std::optional<uint32_t> horizontal_opt = get_epsg_code(gdal_srs, horizontal_node);
        const std::optional<uint32_t> vertical_opt = get_epsg_code(gdal_srs, "VERT_CS");
        if (horizontal_opt && vertical_opt) {
            return fmt::format("EPSG:{}+{}", *horizontal_opt, *vertical_opt);
        }
    }

    return std::nullopt;
}

std::optional<std::string> friendly_name(const System &srs) {
    const std::optional<std::string_view> name_opt = to_string_view(srs.as_gdal().GetName());
    // PROJ names a system without a name of its own "unknown".
    if (!name_opt || name_opt->empty() || *name_opt == "unknown") {
        return std::nullopt;
    }
    return std::string(*name_opt);
}

std::string format_as(const System &srs) {
    const std::optional<std::string> code_opt = format_epsg_code(srs);
    const std::optional<std::string> name_opt = friendly_name(srs);
    if (code_opt && name_opt) {
        return fmt::format("{} ({})", *code_opt, *name_opt);
    }
    return code_opt.or_else([&] { return name_opt; }).value_or("unknown srs");
}

}
