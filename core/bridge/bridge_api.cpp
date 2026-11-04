#include "bridge/bridge_api.hpp"

namespace scand3d::bridge {
namespace {
// Bump together with the app's versionName; the bridge stays a skeleton until Phase 1.
constexpr char kCoreVersion[] = "0.1.0";
}  // namespace

const char* coreVersion() {
    return kCoreVersion;
}

}  // namespace scand3d::bridge
