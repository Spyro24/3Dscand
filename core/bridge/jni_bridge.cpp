#include <jni.h>

#include <cmath>
#include <vector>

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

namespace {
// Copies a Kotlin byte array into a vector of unsigned bytes; empty when the array is null.
std::vector<unsigned char> readGray(JNIEnv* env, jbyteArray source) {
    std::vector<unsigned char> gray;
    if (source == nullptr) {
        return gray;
    }
    const jsize length = env->GetArrayLength(source);
    gray.resize(static_cast<std::size_t>(length));
    env->GetByteArrayRegion(source, 0, length, reinterpret_cast<jbyte*>(gray.data()));
    return gray;
}
}  // namespace

extern "C" JNIEXPORT jlong JNICALL
Java_com_scand3d_core_bridge_NativeCore_nativeReconstructionCreate(JNIEnv* /*env*/,
                                                                   jobject /*instance*/) {
    return static_cast<jlong>(scand3d::bridge::reconstructionCreate());
}

extern "C" JNIEXPORT void JNICALL
Java_com_scand3d_core_bridge_NativeCore_nativeReconstructionDestroy(JNIEnv* /*env*/,
                                                                    jobject /*instance*/,
                                                                    jlong handle) {
    scand3d::bridge::reconstructionDestroy(
        static_cast<scand3d::bridge::ReconstructionHandle>(handle));
}

extern "C" JNIEXPORT jint JNICALL
Java_com_scand3d_core_bridge_NativeCore_nativeReconstructionAddView(
    JNIEnv* env, jobject /*instance*/, jlong handle, jbyteArray gray, jint width_px, jint height_px,
    jdouble focal_px, jdouble cx_px, jdouble cy_px) {
    const std::vector<unsigned char> pixels = readGray(env, gray);
    if (pixels.empty() || width_px <= 0 || height_px <= 0 ||
        pixels.size() != static_cast<std::size_t>(width_px) * static_cast<std::size_t>(height_px)) {
        return -1;
    }
    return scand3d::bridge::reconstructionAddView(
        static_cast<scand3d::bridge::ReconstructionHandle>(handle), pixels.data(), width_px,
        height_px, focal_px, cx_px, cy_px);
}

extern "C" JNIEXPORT jint JNICALL
Java_com_scand3d_core_bridge_NativeCore_nativeReconstructionRun(JNIEnv* /*env*/,
                                                                jobject /*instance*/,
                                                                jlong handle) {
    return scand3d::bridge::reconstructionRun(
        static_cast<scand3d::bridge::ReconstructionHandle>(handle));
}

extern "C" JNIEXPORT jint JNICALL
Java_com_scand3d_core_bridge_NativeCore_nativeReconstructionPointCount(JNIEnv* /*env*/,
                                                                       jobject /*instance*/,
                                                                       jlong handle) {
    return scand3d::bridge::reconstructionPointCount(
        static_cast<scand3d::bridge::ReconstructionHandle>(handle));
}

extern "C" JNIEXPORT jint JNICALL
Java_com_scand3d_core_bridge_NativeCore_nativeReconstructionCopyPoints(
    JNIEnv* env, jobject /*instance*/, jlong handle, jdoubleArray out_positions) {
    if (out_positions == nullptr) {
        return 0;
    }
    const jsize length = env->GetArrayLength(out_positions);
    const int max_points = length / 3;
    if (max_points <= 0) {
        return 0;
    }
    std::vector<double> positions(static_cast<std::size_t>(max_points) * 3);
    const int written = scand3d::bridge::reconstructionCopyPoints(
        static_cast<scand3d::bridge::ReconstructionHandle>(handle), positions.data(),
        max_points);
    if (written <= 0) {
        return 0;
    }
    env->SetDoubleArrayRegion(out_positions, 0, written * 3, positions.data());
    return written;
}

extern "C" JNIEXPORT jint JNICALL
Java_com_scand3d_core_bridge_NativeCore_nativeReconstructionTrackPose(
    JNIEnv* env, jobject /*instance*/, jlong handle, jbyteArray gray, jint width_px, jint height_px,
    jdouble focal_px, jdouble cx_px, jdouble cy_px, jdoubleArray out_rotation,
    jdoubleArray out_centre_m) {
    if (out_rotation == nullptr || out_centre_m == nullptr ||
        env->GetArrayLength(out_rotation) < 9 || env->GetArrayLength(out_centre_m) < 3) {
        return 0;
    }
    const std::vector<unsigned char> pixels = readGray(env, gray);
    if (pixels.empty() || width_px <= 0 || height_px <= 0 ||
        pixels.size() != static_cast<std::size_t>(width_px) * static_cast<std::size_t>(height_px)) {
        return 0;
    }
    double rotation[9] = {};
    double centre[3] = {};
    const int inliers = scand3d::bridge::reconstructionTrackPose(
        static_cast<scand3d::bridge::ReconstructionHandle>(handle), pixels.data(), width_px,
        height_px, focal_px, cx_px, cy_px, rotation, centre);
    if (inliers <= 0) {
        return 0;
    }
    env->SetDoubleArrayRegion(out_rotation, 0, 9, rotation);
    env->SetDoubleArrayRegion(out_centre_m, 0, 3, centre);
    return inliers;
}

extern "C" JNIEXPORT jlong JNICALL
Java_com_scand3d_core_bridge_NativeCore_nativeOverlayCreate(JNIEnv* /*env*/, jobject /*instance*/) {
    return static_cast<jlong>(scand3d::bridge::overlayCreate());
}

extern "C" JNIEXPORT void JNICALL
Java_com_scand3d_core_bridge_NativeCore_nativeOverlayDestroy(JNIEnv* /*env*/, jobject /*instance*/,
                                                             jlong handle) {
    scand3d::bridge::overlayDestroy(static_cast<scand3d::bridge::OverlayHandle>(handle));
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_scand3d_core_bridge_NativeCore_nativeOverlaySetPoints(JNIEnv* env, jobject /*instance*/,
                                                               jlong handle,
                                                               jdoubleArray positions_m,
                                                               jfloatArray rgb) {
    if (positions_m == nullptr || rgb == nullptr) {
        return JNI_FALSE;
    }
    const jsize position_count = env->GetArrayLength(positions_m);
    if (position_count % 3 != 0 || env->GetArrayLength(rgb) != position_count) {
        return JNI_FALSE;
    }
    const jsize point_count = position_count / 3;
    std::vector<double> positions(static_cast<std::size_t>(position_count));
    std::vector<float> colours(static_cast<std::size_t>(position_count));
    env->GetDoubleArrayRegion(positions_m, 0, position_count, positions.data());
    env->GetFloatArrayRegion(rgb, 0, position_count, colours.data());
    const bool stored = scand3d::bridge::overlaySetPoints(
        static_cast<scand3d::bridge::OverlayHandle>(handle), positions.data(), colours.data(),
        static_cast<int>(point_count));
    return stored ? JNI_TRUE : JNI_FALSE;
}

extern "C" JNIEXPORT jint JNICALL
Java_com_scand3d_core_bridge_NativeCore_nativeOverlayProject(
    JNIEnv* env, jobject /*instance*/, jlong handle, jdoubleArray rotation, jdoubleArray centre_m,
    jdouble focal_px, jdouble cx_px, jdouble cy_px, jint image_width_px, jint image_height_px,
    jfloatArray out_vertices) {
    if (rotation == nullptr || centre_m == nullptr || out_vertices == nullptr ||
        env->GetArrayLength(rotation) < 9 || env->GetArrayLength(centre_m) < 3) {
        return 0;
    }
    const jsize out_length = env->GetArrayLength(out_vertices);
    const int max_vertices = out_length / scand3d::bridge::kOverlayVertexValueCount;
    if (max_vertices <= 0) {
        return 0;
    }

    scand3d::bridge::OverlayCamera camera{};
    env->GetDoubleArrayRegion(rotation, 0, 9, camera.rotation);
    env->GetDoubleArrayRegion(centre_m, 0, 3, camera.centre_m);
    camera.focal_px = focal_px;
    camera.cx_px = cx_px;
    camera.cy_px = cy_px;
    camera.image_width_px = image_width_px;
    camera.image_height_px = image_height_px;

    std::vector<float> vertices(
        static_cast<std::size_t>(max_vertices) *
        static_cast<std::size_t>(scand3d::bridge::kOverlayVertexValueCount));
    const int written = scand3d::bridge::overlayProject(
        static_cast<scand3d::bridge::OverlayHandle>(handle), camera, vertices.data(),
        max_vertices);
    if (written <= 0) {
        return 0;
    }
    env->SetFloatArrayRegion(out_vertices, 0, written * scand3d::bridge::kOverlayVertexValueCount,
                             vertices.data());
    return written;
}

extern "C" JNIEXPORT jint JNICALL
Java_com_scand3d_core_bridge_NativeCore_nativeOverlaySampleColors(
    JNIEnv* env, jobject /*instance*/, jlong handle, jdoubleArray rotation, jdoubleArray centre_m,
    jdouble focal_px, jdouble cx_px, jdouble cy_px, jint image_width_px, jint image_height_px,
    jbyteArray rgb, jint keyframe_width_px, jint keyframe_height_px) {
    if (rotation == nullptr || centre_m == nullptr || rgb == nullptr ||
        env->GetArrayLength(rotation) < 9 || env->GetArrayLength(centre_m) < 3 ||
        keyframe_width_px <= 0 || keyframe_height_px <= 0) {
        return 0;
    }
    const jsize expected = keyframe_width_px * keyframe_height_px * 3;
    if (env->GetArrayLength(rgb) != expected) {
        return 0;
    }

    scand3d::bridge::OverlayCamera camera{};
    env->GetDoubleArrayRegion(rotation, 0, 9, camera.rotation);
    env->GetDoubleArrayRegion(centre_m, 0, 3, camera.centre_m);
    camera.focal_px = focal_px;
    camera.cx_px = cx_px;
    camera.cy_px = cy_px;
    camera.image_width_px = image_width_px;
    camera.image_height_px = image_height_px;

    std::vector<unsigned char> pixels(static_cast<std::size_t>(expected));
    env->GetByteArrayRegion(rgb, 0, expected, reinterpret_cast<jbyte*>(pixels.data()));
    return scand3d::bridge::overlaySampleColors(
        static_cast<scand3d::bridge::OverlayHandle>(handle), camera, pixels.data(),
        keyframe_width_px, keyframe_height_px);
}
