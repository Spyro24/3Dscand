#include <gtest/gtest.h>

#include <cstdio>
#include <cstring>

#include "bridge/bridge_api.hpp"

namespace {

// The UI displays this string, so it must always be present and parseable.
TEST(BridgeApi, ReportsNonEmptyVersion) {
    const char* version = scand3d::bridge::coreVersion();
    ASSERT_NE(version, nullptr);
    EXPECT_GT(std::strlen(version), 0u);
}

TEST(BridgeApi, VersionIsSemantic) {
    const char* version = scand3d::bridge::coreVersion();
    ASSERT_NE(version, nullptr);

    int major = -1;
    int minor = -1;
    int patch = -1;
    EXPECT_EQ(3, std::sscanf(version, "%d.%d.%d", &major, &minor, &patch));
    EXPECT_GE(major, 0);
    EXPECT_GE(minor, 0);
    EXPECT_GE(patch, 0);
}

}  // namespace
