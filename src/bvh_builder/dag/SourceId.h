#pragma once

#include <variant>

#include "dag_id.h"

namespace dag {

// The id of a group, or of a cluster no group lists.
using SourceId = std::variant<GroupId, ClusterId>;

} // namespace dag
