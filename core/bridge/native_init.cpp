#include "bridge/native_init.hpp"

#include <android/log.h>

namespace scand3d::bridge {
namespace {
constexpr char kLogTag[] = "scand3d-native";
}  // namespace

void onLibraryLoad() {
    __android_log_print(ANDROID_LOG_INFO, kLogTag, "libscand3d_core loaded");
}

}  // namespace scand3d::bridge
