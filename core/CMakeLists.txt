cmake_minimum_required(VERSION 3.22.1)

project(scand3d_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# One shared library holds the whole native core; the Kotlin layer reaches it through the
# single JNI bridge in core/bridge (agents.md section 3). Per-ABI packaging is controlled by
# the app module's abiFilters.
add_library(scand3d_core SHARED
    bridge/bridge_api.cpp
    bridge/jni_bridge.cpp
    bridge/native_init.cpp
    feature/brief_descriptor.cpp
    feature/fast_detector.cpp
    feature/feature_extractor.cpp
    feature/feature_store.cpp
    feature/gray_image.cpp
    feature/matcher.cpp
    geometry/linalg.cpp
    geometry/two_view.cpp
    overlay/overlay_scene.cpp
    overlay/point_color.cpp
    sensors/keyframe_prior.cpp
    sensors/mahony_filter.cpp
    sensors/tilt_orientation.cpp
    sfm/bundle_adjustment.cpp
    sfm/keyframe_selection.cpp
    sfm/map_store.cpp
    sfm/pipeline.cpp
    sfm/pnp.cpp
    sfm/pose.cpp
    sfm/report.cpp
    sfm/session.cpp
    sfm/triangulation.cpp
)

target_include_directories(scand3d_core PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")

find_library(ANDROID_LOG_LIB log REQUIRED)
target_link_libraries(scand3d_core PRIVATE "${ANDROID_LOG_LIB}")
