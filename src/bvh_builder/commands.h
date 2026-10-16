#pragma once

#include "error/Result.h"

#include "cli.h"

// Builds the bvh over the dag at the input path, writing it to the output path.
[[nodiscard]]
alp::Result<void> build_bvh(const cli::BuildArgs &args);

// Reports the cut cost per camera.
[[nodiscard]]
alp::Result<void> evaluate_bvh(const cli::EvaluateArgs &args);
