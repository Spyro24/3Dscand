#include <jni.h>

#include "bridge/bridge_api.hpp"
#include "bridge/native_init.hpp"

// Entry point of the single JNI bridge between Kotlin and the native core.
JNIEXPORT jint JNI_OnLoad(JavaVM* /*vm*/, void* /*reserved*/) {
    scand3d::bridge::onLibraryLoad();
    return JNI_VERSION_1_6;
}

// Mirrors NativeCore.nativeCoreVersion() in com.scand3d.core.bridge. NativeCore is a Kotlin
// object, so the second parameter is the singleton instance rather than a jclass.
extern "C" JNIEXPORT jstring JNICALL
Java_com_scand3d_core_bridge_NativeCore_nativeCoreVersion(JNIEnv* env, jobject /*instance*/) {
    return env->NewStringUTF(scand3d::bridge::coreVersion());
}
