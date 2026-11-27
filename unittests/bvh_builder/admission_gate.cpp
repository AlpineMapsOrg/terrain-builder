#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "frontier_helpers.h"
#include "node_header_helpers.h"

#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "frontier/layout.h"
#include "gated_ploc/AdmissionGate.h"
#include "gated_ploc/options.h"
#include "morton/utils.h"

namespace {

// A cube of side `extent` at the given code.
frontier::Entry make_entry(const double extent, const morton::Code morton) {
    return frontier::Entry{
        .morton = morton,
        .header = make_node_header({0.0, 0.0}, bvh::Bounds{glm::dvec3{0.0}, glm::dvec3{extent}}),
        .ref = bvh::Reference{},
    };
}

// Regions of four, each admitting up to its second smallest extent.
const gated_ploc::AdmissionOptions OPTIONS{.region_size = 4, .quantile = 0.0, .tolerance = 1.0};

}

TEST_CASE("AdmissionGate splits each full region, each side in input order", "[gated_ploc][AdmissionGate]") {
    // The second smallest extent is 2, so 20 and 30 are admitted. The fifth entry starts a region that is not full yet.
    const std::vector<frontier::Entry> entries{
        make_entry(3.0, 10),
        make_entry(1.0, 20),
        make_entry(2.0, 30),
        make_entry(5.0, 40),
        make_entry(1.0, 50),
    };

    std::vector<morton::Code> admitted;
    std::vector<morton::Code> held;
    bool admitted_finished = false;
    bool held_finished = false;
    gated_ploc::detail::AdmissionGate gate(CollectingSink{admitted, admitted_finished}, CollectingSink{held, held_finished}, OPTIONS);

    REQUIRE_VALUE(gate.push(entries));

    CHECK(admitted == std::vector<morton::Code>{20, 30});
    CHECK(held == std::vector<morton::Code>{10, 40});
    CHECK_FALSE(admitted_finished);
    CHECK_FALSE(held_finished);
}

TEST_CASE("AdmissionGate finish passes on the partial region and ends both streams", "[gated_ploc][AdmissionGate]") {
    // The second smallest extent is 2, so 10 and 30 are admitted.
    const std::vector<frontier::Entry> entries{
        make_entry(1.0, 10),
        make_entry(3.0, 20),
        make_entry(2.0, 30),
    };

    std::vector<morton::Code> admitted;
    std::vector<morton::Code> held;
    bool admitted_finished = false;
    bool held_finished = false;
    gated_ploc::detail::AdmissionGate gate(CollectingSink{admitted, admitted_finished}, CollectingSink{held, held_finished}, OPTIONS);

    REQUIRE_VALUE(gate.push(entries));
    REQUIRE(admitted.empty());
    REQUIRE(held.empty());
    REQUIRE_VALUE(std::move(gate).finish());

    CHECK(admitted == std::vector<morton::Code>{10, 30});
    CHECK(held == std::vector<morton::Code>{20});
    CHECK(admitted_finished);
    CHECK(held_finished);
}
