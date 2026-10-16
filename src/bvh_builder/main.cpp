#include <cstdlib>
#include <exception>
#include <variant>

#include "cli.h"
#include "commands.h"
#include "error/Result.h"
#include "error/format.h"
#include "error/log.h"
#include "log.h"
#include "variant_utils.h"

int main(int argc, char **argv) {
    const cli::Args args = cli::parse(argc, argv);

    const auto log_level = std::visit([](const auto &a) { return a.log_level; }, args);
    Log::init(log_level);

    try {
        const alp::Result<void> result = match(args,
            [](const cli::BuildArgs &a) { return build_bvh(a); });
        if (!result.has_value()) {
            LOG_UNEXPECTED(result.error());
            return EXIT_FAILURE;
        }
        return EXIT_SUCCESS;
    } catch (const std::exception &e) {
        LOG_ERROR("{}", e.what());
        return EXIT_FAILURE;
    }
}
