cmake_minimum_required(VERSION 3.22)

# Host build of the C++ unit tests. The app ships an arm64-v8a library that an x86_64 host
# cannot load, so the core sources are compiled for the host and linked against googletest
# instead. Nothing here ends up in the APK.
project(scand3d_native_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
    synthetic/synthetic_scene.cpp
)
target_include_directories(scand3d_test_util PUBLIC
    "${SCAND3D_CORE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

include(GoogleTest)

add_executable(bridge_api_test
    native/bridge_api_test.cpp
)
target_link_libraries(bridge_api_test PRIVATE scand3d_test_util GTest::gtest_main)
gtest_discover_tests(bridge_api_test)

add_executable(synthetic_scene_test
    native/synthetic_scene_test.cpp
)
target_link_libraries(synthetic_scene_test PRIVATE scand3d_test_util GTest::gtest_main)
gtest_discover_tests(synthetic_scene_test)
