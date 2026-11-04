cmake_minimum_required(VERSION 3.22)

# Host build of the C++ unit tests. The app ships an arm64-v8a library that an x86_64 host
# cannot load, so the core sources are compiled for the host and linked against googletest
# instead. Nothing here ends up in the APK.
project(scand3d_native_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# The host tests and the desktop report tools run the same numeric work as the app, so they build
# optimised by default: an unoptimised build is several times slower and made timing numbers
# meaningless (Phase 6 reported 100 views far outside the budget for that reason alone).
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

set(SCAND3D_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../core")

enable_testing()

include(FetchContent)
FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
    GIT_SHALLOW TRUE
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Everything the host tests link against: the core sources under test and the synthetic scene
# utility. Each test executable is a thin wrapper over this library.
add_library(scand3d_test_util STATIC
    "${SCAND3D_CORE_DIR}/bridge/bridge_api.cpp"
    "${SCAND3D_CORE_DIR}/feature/brief_descriptor.cpp"
    "${SCAND3D_CORE_DIR}/feature/fast_detector.cpp"
    "${SCAND3D_CORE_DIR}/feature/feature_extractor.cpp"
    "${SCAND3D_CORE_DIR}/feature/feature_store.cpp"
    "${SCAND3D_CORE_DIR}/feature/gray_image.cpp"
    "${SCAND3D_CORE_DIR}/feature/matcher.cpp"
    "${SCAND3D_CORE_DIR}/geometry/linalg.cpp"
    "${SCAND3D_CORE_DIR}/geometry/two_view.cpp"
    "${SCAND3D_CORE_DIR}/mvs/consistency.cpp"
    "${SCAND3D_CORE_DIR}/mvs/fusion.cpp"
    "${SCAND3D_CORE_DIR}/mvs/patchmatch.cpp"
    "${SCAND3D_CORE_DIR}/mvs/pipeline.cpp"
    "${SCAND3D_CORE_DIR}/mvs/types.cpp"
    "${SCAND3D_CORE_DIR}/overlay/overlay_scene.cpp"
    "${SCAND3D_CORE_DIR}/overlay/point_color.cpp"
    "${SCAND3D_CORE_DIR}/sensors/keyframe_prior.cpp"
    "${SCAND3D_CORE_DIR}/sensors/mahony_filter.cpp"
    "${SCAND3D_CORE_DIR}/sensors/tilt_orientation.cpp"
    "${SCAND3D_CORE_DIR}/sfm/bundle_adjustment.cpp"
    "${SCAND3D_CORE_DIR}/sfm/keyframe_selection.cpp"
    "${SCAND3D_CORE_DIR}/sfm/map_store.cpp"
    "${SCAND3D_CORE_DIR}/sfm/pipeline.cpp"
    "${SCAND3D_CORE_DIR}/sfm/pnp.cpp"
    "${SCAND3D_CORE_DIR}/sfm/pose.cpp"
    "${SCAND3D_CORE_DIR}/sfm/report.cpp"
    "${SCAND3D_CORE_DIR}/sfm/session.cpp"
    "${SCAND3D_CORE_DIR}/sfm/tracker.cpp"
    "${SCAND3D_CORE_DIR}/sfm/triangulation.cpp"
    synthetic/sfm_scan.cpp
    synthetic/synthetic_scene.cpp
    tools/match_overlay.cpp
    tools/pgm.cpp
)
target_include_directories(scand3d_test_util PUBLIC
    "${SCAND3D_CORE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}"
)
find_package(Threads REQUIRED)
target_link_libraries(scand3d_test_util PUBLIC Threads::Threads)

include(GoogleTest)

add_executable(bridge_api_test
    native/bridge_api_test.cpp
)
target_link_libraries(bridge_api_test PRIVATE scand3d_test_util GTest::gtest_main)
gtest_discover_tests(bridge_api_test)

add_executable(imu_fusion_test
    native/imu_fusion_test.cpp
)
target_link_libraries(imu_fusion_test PRIVATE scand3d_test_util GTest::gtest_main)
gtest_discover_tests(imu_fusion_test)

add_executable(feature_test
    native/feature_test.cpp
)
target_link_libraries(feature_test PRIVATE scand3d_test_util GTest::gtest_main)
gtest_discover_tests(feature_test)

add_executable(geometry_test
    native/geometry_test.cpp
)
target_link_libraries(geometry_test PRIVATE scand3d_test_util GTest::gtest_main)
gtest_discover_tests(geometry_test)

add_executable(sfm_test
    native/sfm_test.cpp
)
target_link_libraries(sfm_test PRIVATE scand3d_test_util GTest::gtest_main)
gtest_discover_tests(sfm_test)

add_executable(match_tool_test
    native/match_tool_test.cpp
)
target_link_libraries(match_tool_test PRIVATE scand3d_test_util GTest::gtest_main)
gtest_discover_tests(match_tool_test)

# Desktop-only debugging tool (roadmap 3.5); never part of the APK.
add_executable(match_visualizer
    tools/match_visualizer.cpp
)
target_link_libraries(match_visualizer PRIVATE scand3d_test_util)

# Desktop-only reconstruction report (roadmap 4.6); never part of the APK.
add_executable(sfm_report
    tools/sfm_report.cpp
)
target_link_libraries(sfm_report PRIVATE scand3d_test_util)

add_executable(overlay_test
    native/overlay_test.cpp
)
target_link_libraries(overlay_test PRIVATE scand3d_test_util GTest::gtest_main)
gtest_discover_tests(overlay_test)

add_executable(session_test
    native/session_test.cpp
)
target_link_libraries(session_test PRIVATE scand3d_test_util GTest::gtest_main)
gtest_discover_tests(session_test)

add_executable(point_color_test
    native/point_color_test.cpp
)
target_link_libraries(point_color_test PRIVATE scand3d_test_util GTest::gtest_main)
gtest_discover_tests(point_color_test)

add_executable(synthetic_scene_test
    native/synthetic_scene_test.cpp
)
target_link_libraries(synthetic_scene_test PRIVATE scand3d_test_util GTest::gtest_main)
gtest_discover_tests(synthetic_scene_test)

add_executable(mvs_test
    native/mvs_test.cpp
)
target_link_libraries(mvs_test PRIVATE scand3d_test_util GTest::gtest_main)
gtest_discover_tests(mvs_test)

# Desktop-only dense reconstruction report (roadmap 6.1-6.5); never part of the APK.
add_executable(mvs_report
    tools/mvs_report.cpp
)
target_link_libraries(mvs_report PRIVATE scand3d_test_util)
