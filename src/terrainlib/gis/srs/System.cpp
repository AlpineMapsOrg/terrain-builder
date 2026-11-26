/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Martin Braunsperger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *****************************************************************************/

#include "gis/srs/System.h"

#include <cstdlib>
#include <memory>
#include <string_view>
#include <utility>

#include <cpl_conv.h>
#include <fmt/format.h>
#include <libassert/assert.hpp>

#include "enumerate.h"
#include "error/Context.h"
#include "error/try.h"
#include "error/utils.h"
#include "gis/gdal.h"
#include "string_utils.h"

namespace gis::srs {

namespace {

// The EPSG code of the part of `srs` at `wkt1_path`, such as "DATUM" or "COMPD_CS|VERT_CS", or of `srs` itself if the path is null.
std::optional<uint32_t> get_epsg_code(const OGRSpatialReference &srs, const char *wkt1_path) {
    if (to_string_view(srs.GetAuthorityName(wkt1_path)) != "EPSG") {
        return std::nullopt;
    }

    return to_string_view(srs.GetAuthorityCode(wkt1_path)).and_then([](const std::string_view text) { return from_chars<uint32_t>(text); });
}

AxisDirection to_axis_direction(const OGRAxisOrientation orientation) {
    switch (orientation) {
    case OAO_East:
        return AxisDirection::East;
    case OAO_West:
        return AxisDirection::West;
    case OAO_North:
        return AxisDirection::North;
    case OAO_South:
        return AxisDirection::South;
    case OAO_Up:
        return AxisDirection::Up;
    case OAO_Down:
        return AxisDirection::Down;
    case OAO_Other:
        return AxisDirection::Other;
    }
    return AxisDirection::Other;
}

AxisDirection reverse(const AxisDirection direction) {
    switch (direction) {
    case AxisDirection::East:
        return AxisDirection::West;
    case AxisDirection::West:
        return AxisDirection::East;
    case AxisDirection::North:
        return AxisDirection::South;
    case AxisDirection::South:
        return AxisDirection::North;
    case AxisDirection::Up:
        return AxisDirection::Down;
    case AxisDirection::Down:
        return AxisDirection::Up;
    case AxisDirection::Other:
        return AxisDirection::Other;
    }
    UNREACHABLE("invalid direction", direction);
}

}

System::System(OGRSpatialReference srs) : System(std::make_shared<const OGRSpatialReference>(std::move(srs))) {}

System::System(std::shared_ptr<const OGRSpatialReference> srs) : _srs(std::move(srs)) {
    DEBUG_ASSERT(this->_srs != nullptr);
    DEBUG_ASSERT(!this->_srs->IsEmpty());
}

// An empty owner makes the pointer non-owning.
System::System(const OGRSpatialReference *srs) : System(std::shared_ptr<const OGRSpatialReference>(std::shared_ptr<void>(), srs)) {}

System System::clone() const {
    return System(OGRSpatialReference(*this->_srs));
}

std::optional<uint32_t> System::epsg_code() const {
    if (const std::optional<uint32_t> code_opt = get_epsg_code(*this->_srs, nullptr)) {
        return code_opt;
    }

    OGRSpatialReference identified = *this->_srs;
    identified.AutoIdentifyEPSG();
    return get_epsg_code(identified, nullptr);
}

Kind System::kind() const {
    if (this->_srs->IsGeocentric()) {
        return Kind::Geocentric;
    }
    // A compound system is projected or geographic if its horizontal part is, so these come before the vertical check.
    if (this->_srs->IsProjected()) {
        return Kind::Projected;
    }
    if (this->_srs->IsGeographic()) {
        return Kind::Geographic;
    }
    if (this->_srs->IsLocal()) {
        return Kind::Engineering;
    }
    if (this->_srs->IsVertical()) {
        return Kind::Vertical;
    }
    return Kind::Other;
}

alp::Result<Ellipsoid, NotGeodetic> System::ellipsoid() const {
    if (!is_geodetic(this->kind())) {
        return alp::make_error(NotGeodetic{});
    }

    // Every geodetic system has an ellipsoid.
    OGRErr semi_major_error = OGRERR_NONE;
    OGRErr semi_minor_error = OGRERR_NONE;
    const Ellipsoid ellipsoid{
        .semi_major_metres = this->_srs->GetSemiMajor(&semi_major_error),
        .semi_minor_metres = this->_srs->GetSemiMinor(&semi_minor_error)};
    DEBUG_ASSERT(semi_major_error == OGRERR_NONE);
    DEBUG_ASSERT(semi_minor_error == OGRERR_NONE);
    return ellipsoid;
}

alp::Result<double, NotGeodetic> System::prime_meridian_degrees() const {
    if (!is_geodetic(this->kind())) {
        return alp::make_error(NotGeodetic{});
    }

    return this->_srs->GetPrimeMeridian();
}

std::vector<Axis> System::axes() const {
    const bool is_geographic = this->kind() == Kind::Geographic;

    // Entry i of the mapping is the data axis that holds axis i of the system, counted from 1 and negative for a reversed sign.
    const std::vector<int> &mapping = this->_srs->GetDataAxisToSRSAxisMapping();
    std::vector<Axis> axes(mapping.size());
    for (const auto [srs_axis, data_axis] : enumerate<int>(mapping)) {
        const size_t index = std::abs(data_axis) - 1;
        DEBUG_ASSERT(data_axis != 0 && index < axes.size());

        OGRAxisOrientation orientation = OAO_Other;
        double si_size = 0;
        const char *name = this->_srs->GetAxis(nullptr, srs_axis, &orientation, &si_size);
        DEBUG_ASSERT(name != nullptr);

        const AxisDirection direction = to_axis_direction(orientation);
        // A geographic system's third axis, and the vertical axis of a compound one, is a height.
        const Quantity quantity = is_geographic && srs_axis < 2 ? Quantity::Angle : Quantity::Length;
        axes[index] = Axis{
            .name = std::string(name),
            .direction = data_axis < 0 ? reverse(direction) : direction,
            .unit = Unit{quantity, si_size}};
    }
    return axes;
}

alp::Result<std::string, OgrError> System::to_wkt(const WktVersion version) const {
    const char *format = nullptr;
    switch (version) {
    case WktVersion::Gdal1:
        format = "FORMAT=WKT1_GDAL";
        break;
    case WktVersion::Iso2019:
        format = "FORMAT=WKT2_2019";
        break;
    }
    ASSERT(format != nullptr, "invalid WKT version", version);
    const char *const options[] = {format, nullptr};
    char *raw_wkt = nullptr;
    const OGRErr error = this->_srs->exportToWkt(&raw_wkt, options);
    const std::unique_ptr<char, decltype(&CPLFree)> wkt(raw_wkt, &CPLFree);

    ALP_TRY(to_result(error));
    return std::string(wkt.get());
}

alp::Result<System, OgrError> from_epsg(const uint32_t code) {
    gdal::initialize_once();
    const auto context = alp::make_context("importing a spatial reference from EPSG code {}", code);

    OGRSpatialReference srs;
    ALP_TRY_CONTEXT(to_result(srs.importFromEPSG(code)), context);
    srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
    return System(std::move(srs));
}

alp::Result<System, OgrError> from_user_input(const std::string &input) {
    gdal::initialize_once();
    const auto context = alp::make_context("parsing a spatial reference from {}", input);

    OGRSpatialReference srs;
    ALP_TRY_CONTEXT(to_result(srs.SetFromUserInput(input.c_str())), context);
    srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
    return System(std::move(srs));
}

std::optional<std::string> format_epsg_code(const System &srs) {
    if (const std::optional<uint32_t> code_opt = srs.epsg_code()) {
        return fmt::format("EPSG:{}", *code_opt);
    }

    if (srs.is_compound()) {
        const OGRSpatialReference &gdal_srs = srs.as_gdal();
        // The search for a node goes depth first, so GEOGCS would find the base of a projected horizontal part.
        const char *horizontal_node = srs.kind() == Kind::Projected ? "PROJCS" : "GEOGCS";
        ALP_TRY(const uint32_t horizontal_code, get_epsg_code(gdal_srs, horizontal_node));
        ALP_TRY(const uint32_t vertical_code, get_epsg_code(gdal_srs, "VERT_CS"));
        return fmt::format("EPSG:{}+{}", horizontal_code, vertical_code);
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
