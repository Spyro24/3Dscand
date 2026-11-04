#include <jni.h>

#include "bridge/native_init.hpp"

// Entry point of the single JNI bridge between Kotlin and the native core.
JNIEXPORT jint JNI_OnLoad(JavaVM* /*vm*/, void* /*reserved*/) {
    scand3d::bridge::onLibraryLoad();
    return JNI_VERSION_1_6;
}
