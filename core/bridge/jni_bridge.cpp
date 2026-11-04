#include <jni.h>

#include <cmath>

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

extern "C" JNIEXPORT jlong JNICALL
Java_com_scand3d_core_bridge_NativeCore_nativeImuCreate(JNIEnv* /*env*/, jobject /*instance*/,
                                                        jdouble sample_rate_hz) {
    return static_cast<jlong>(scand3d::bridge::imuCreate(sample_rate_hz));
}

extern "C" JNIEXPORT void JNICALL
Java_com_scand3d_core_bridge_NativeCore_nativeImuDestroy(JNIEnv* /*env*/, jobject /*instance*/,
                                                         jlong handle) {
    scand3d::bridge::imuDestroy(static_cast<scand3d::bridge::ImuHandle>(handle));
}

extern "C" JNIEXPORT void JNICALL
Java_com_scand3d_core_bridge_NativeCore_nativeImuReset(JNIEnv* /*env*/, jobject /*instance*/,
                                                       jlong handle) {
    scand3d::bridge::imuReset(static_cast<scand3d::bridge::ImuHandle>(handle));
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_scand3d_core_bridge_NativeCore_nativeImuUpdate(JNIEnv* env, jobject /*instance*/,
                                                        jlong handle, jdoubleArray values) {
    if (values == nullptr ||
        env->GetArrayLength(values) != scand3d::bridge::kImuSampleValueCount) {
        return JNI_FALSE;
    }
    jdouble buffer[scand3d::bridge::kImuSampleValueCount];
    env->GetDoubleArrayRegion(values, 0, scand3d::bridge::kImuSampleValueCount, buffer);
    const bool updated = scand3d::bridge::imuUpdate(
        static_cast<scand3d::bridge::ImuHandle>(handle), buffer,
        scand3d::bridge::kImuSampleValueCount);
    return updated ? JNI_TRUE : JNI_FALSE;
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_scand3d_core_bridge_NativeCore_nativeImuOrientation(JNIEnv* env, jobject /*instance*/,
                                                             jlong handle, jdoubleArray out) {
    if (out == nullptr || env->GetArrayLength(out) < 4) {
        return JNI_FALSE;
    }
    double quaternion[4];
    if (!scand3d::bridge::imuOrientation(
            static_cast<scand3d::bridge::ImuHandle>(handle), quaternion)) {
        return JNI_FALSE;
    }
    env->SetDoubleArrayRegion(out, 0, 4, quaternion);
    return JNI_TRUE;
}

namespace {
// Reads exactly four doubles from a Kotlin array; false when the array is missing or short.
bool readQuaternion(JNIEnv* env, jdoubleArray source, double out[4]) {
    if (source == nullptr || env->GetArrayLength(source) < 4) {
        return false;
    }
    env->GetDoubleArrayRegion(source, 0, 4, out);
    return true;
}
}  // namespace

extern "C" JNIEXPORT jdouble JNICALL
Java_com_scand3d_core_bridge_NativeCore_nativeAngularDisplacementRad(
    JNIEnv* env, jobject /*instance*/, jdoubleArray a, jdoubleArray b) {
    double first[4];
    double second[4];
    if (!readQuaternion(env, a, first) || !readQuaternion(env, b, second)) {
        return static_cast<jdouble>(std::nan(""));
    }
    return scand3d::bridge::angularDisplacementRad(first, second);
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_scand3d_core_bridge_NativeCore_nativeKeyframeBaselineAccept(
    JNIEnv* env, jobject /*instance*/, jdoubleArray reference, jdoubleArray candidate,
    jdouble min_baseline_rad, jdouble max_baseline_rad) {
    double first[4];
    double second[4];
    if (!readQuaternion(env, reference, first) || !readQuaternion(env, candidate, second)) {
        return JNI_FALSE;
    }
    const bool accepted = scand3d::bridge::keyframeBaselineAccept(
        first, second, min_baseline_rad, max_baseline_rad);
    return accepted ? JNI_TRUE : JNI_FALSE;
}
