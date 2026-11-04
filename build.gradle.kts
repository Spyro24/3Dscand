// Plugins are declared here so every module resolves the same versions.
plugins {
    alias(libs.plugins.android.application) apply false
    alias(libs.plugins.kotlin.android) apply false
    alias(libs.plugins.kotlin.compose) apply false
}

// Host build of the C++ unit tests. The app's shared library is arm64-v8a only, so the core
// sources are compiled for the host instead of loaded into the test process.
tasks.register<Exec>("nativeTest") {
    group = "verification"
    description = "Builds and runs the gtest suite for the C++ core on the host."
    commandLine(
        "bash",
        "-c",
        "cmake -S tests -B build/native-tests -DCMAKE_BUILD_TYPE=Debug && " +
            "cmake --build build/native-tests --parallel && " +
            "ctest --test-dir build/native-tests --output-on-failure",
    )
}
