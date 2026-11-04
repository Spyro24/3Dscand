#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "bridge/bridge_api.hpp"
#include "sensors/imu_types.hpp"
#include "sensors/keyframe_prior.hpp"
#include "sensors/mahony_filter.hpp"
#include "sensors/tilt_orientation.hpp"

namespace {

using scand3d::sensors::ImuSample;
using scand3d::sensors::Quat;
using scand3d::sensors::Vec3;
using scand3d::sensors::quatAngleRad;
using scand3d::sensors::quatConjugate;
using scand3d::sensors::quatFromAxisAngle;
using scand3d::sensors::quatRotate;
using scand3d::sensors::isFiniteQuat;

constexpr double kGravity = 9.81;
constexpr double kPi = 3.14159265358979323846;

ImuSample levelSample(std::int64_t timestamp_ns) {
    ImuSample sample;
    sample.timestamp_ns = timestamp_ns;
    sample.accel_m_s2 = {0.0, 0.0, kGravity};
    sample.has_accel = true;
    return sample;
}

Vec3 normalized(const Vec3& v) {
    Vec3 copy = v;
    EXPECT_TRUE(scand3d::sensors::normalize(copy));
    return copy;
}

// ---------------------------------------------------------------- quaternion math

TEST(QuatMath, RotationMatchesItsAxisAngle) {
    const Quat yaw = quatFromAxisAngle({0.0, 0.0, 1.0}, 0.5 * kPi);

    // A quarter turn about +Z maps body +X onto world +Y.
    const Vec3 rotated = quatRotate(yaw, {1.0, 0.0, 0.0});
    EXPECT_NEAR(0.0, rotated[0], 1e-12);
    EXPECT_NEAR(1.0, rotated[1], 1e-12);
    EXPECT_NEAR(0.0, rotated[2], 1e-12);

    // The world up axis is unchanged by a yaw.
    const Vec3 up = quatRotate(yaw, scand3d::sensors::kWorldUp);
    EXPECT_NEAR(0.0, up[0], 1e-12);
    EXPECT_NEAR(0.0, up[1], 1e-12);
    EXPECT_NEAR(1.0, up[2], 1e-12);
}

TEST(QuatMath, AngleBetweenKnownRotations) {
    const Quat identity = scand3d::sensors::identityQuat();
    const Quat thirty = quatFromAxisAngle({1.0, 0.0, 0.0}, 30.0 * kPi / 180.0);

    EXPECT_NEAR(30.0 * kPi / 180.0, quatAngleRad(identity, thirty), 1e-9);
    // Symmetric, and identical quaternions have no displacement.
    EXPECT_NEAR(quatAngleRad(identity, thirty), quatAngleRad(thirty, identity), 1e-12);
    EXPECT_NEAR(0.0, quatAngleRad(thirty, thirty), 1e-12);
}

// ---------------------------------------------------------------- Mahony filter

TEST(MahonyFilter, StaysAtRestWhenLevel) {
    scand3d::sensors::MahonyFilter filter;
    ImuSample sample = levelSample(1'000'000'000);
    sample.gyro_rad_s = {0.0, 0.0, 0.0};
    sample.has_gyro = true;

    for (int i = 0; i < 400; ++i) {
        sample.timestamp_ns += 5'000'000;  // 200 Hz
        ASSERT_TRUE(filter.update(sample, 0.005));
    }

    EXPECT_LT(quatAngleRad(scand3d::sensors::identityQuat(), filter.orientation()), 1e-9);
}

TEST(MahonyFilter, IntegratesNinetyDegreesOfYawFromTheGyroscope) {
    scand3d::sensors::MahonyFilter filter;
    ImuSample sample = levelSample(1'500'000'000);
    sample.has_gyro = true;

    const double rate_rad_s = 0.5 * kPi;
    const int steps = 200;
    const double dt_s = 0.005;
    for (int i = 0; i < steps; ++i) {
        sample.timestamp_ns += 5'000'000;
        sample.gyro_rad_s = {0.0, 0.0, rate_rad_s};
        ASSERT_TRUE(filter.update(sample, dt_s));
    }

    // Gravity is unchanged by a yaw, so the accelerometer term must not fight the rotation.
    const Quat expected = quatFromAxisAngle({0.0, 0.0, 1.0}, rate_rad_s * steps * dt_s);
    EXPECT_NEAR(0.0, quatAngleRad(expected, filter.orientation()), 0.01);
}

TEST(MahonyFilter, ConvergesToTheMagnetometerHeading) {
    scand3d::sensors::MahonyFilter filter;
    ImuSample sample = levelSample(2'000'000'000);
    sample.has_gyro = true;
    sample.gyro_rad_s = {0.0, 0.0, 0.0};
    // Magnetic north lies along body +Y, so the filter has to yaw away from identity.
    sample.mag_u_t = {0.0, 48.0, 0.0};
    sample.has_mag = true;

    for (int i = 0; i < 4000; ++i) {
        sample.timestamp_ns += 5'000'000;
        ASSERT_TRUE(filter.update(sample, 0.005));
    }

    const Quat q = filter.orientation();
    EXPECT_TRUE(isFiniteQuat(q));

    // World north expressed in body coordinates must line up with the measured field, which
    // states the result without depending on a yaw sign convention.
    const Vec3 measured = normalized(sample.mag_u_t);
    const Vec3 world_north_in_body = quatRotate(quatConjugate(q), scand3d::sensors::kWorldNorth);
    EXPECT_GT(scand3d::sensors::dot(world_north_in_body, measured), 0.995);
    // The correction turns the phone -90 degrees: the dot product above already fixes the sign,
    // so this only pins the magnitude.
    EXPECT_NEAR(0.5 * kPi, quatAngleRad(scand3d::sensors::identityQuat(), q), 0.02);
}

TEST(MahonyFilter, RejectsNonPositiveSteps) {
    scand3d::sensors::MahonyFilter filter;
    const ImuSample sample = levelSample(1'000'000'000);

    EXPECT_FALSE(filter.update(sample, 0.0));
    EXPECT_FALSE(filter.update(sample, -0.005));
    EXPECT_FALSE(filter.update(sample, std::numeric_limits<double>::quiet_NaN()));
}

// ---------------------------------------------------------------- fallback orientation

TEST(TiltOrientation, LevelPhoneKeepsNorthAligned) {
    Quat q{};
    ASSERT_TRUE(scand3d::sensors::tiltOrientation({0.0, 0.0, kGravity}, nullptr, &q));
    EXPECT_NEAR(0.0, quatAngleRad(scand3d::sensors::identityQuat(), q), 1e-9);
}

TEST(TiltOrientation, RecoversGravityDirectionWhenTilted) {
    const Vec3 accel{-0.4 * kGravity, 0.3 * kGravity, 0.8 * kGravity};
    Quat q{};
    ASSERT_TRUE(scand3d::sensors::tiltOrientation(accel, nullptr, &q));

    // The estimate must map world up onto the direction gravity was measured in.
    const Vec3 up_in_body = quatRotate(quatConjugate(q), scand3d::sensors::kWorldUp);
    const Vec3 measured = normalized(accel);
    EXPECT_NEAR(1.0, scand3d::sensors::dot(up_in_body, measured), 1e-9);
}

TEST(TiltOrientation, UsesTheMagneticHeadingWhenAvailable) {
    const Vec3 accel{0.0, 0.0, kGravity};
    const Vec3 mag{0.0, 40.0, 0.0};
    Quat q{};
    ASSERT_TRUE(scand3d::sensors::tiltOrientation(accel, &mag, &q));

    const Vec3 world_north_in_body = quatRotate(quatConjugate(q), scand3d::sensors::kWorldNorth);
    EXPECT_GT(scand3d::sensors::dot(world_north_in_body, normalized(mag)), 0.999);
}

TEST(TiltOrientation, IgnoresMagneticVectorParallelToGravity) {
    // A magnetometer reading straight down carries no heading; the basis must stay valid.
    const Vec3 accel{0.0, 0.0, kGravity};
    const Vec3 mag{0.0, 0.0, -30.0};
    Quat q{};
    ASSERT_TRUE(scand3d::sensors::tiltOrientation(accel, &mag, &q));
    EXPECT_TRUE(isFiniteQuat(q));

    const Vec3 up_in_body = quatRotate(quatConjugate(q), scand3d::sensors::kWorldUp);
    EXPECT_NEAR(1.0, scand3d::sensors::dot(up_in_body, {0.0, 0.0, 1.0}), 1e-9);
}

TEST(TiltOrientation, FailsInFreeFall) {
    Quat q{};
    EXPECT_FALSE(scand3d::sensors::tiltOrientation({0.0, 0.0, 0.0}, nullptr, &q));
    EXPECT_FALSE(scand3d::sensors::tiltOrientation({0.0, 0.0, kGravity}, nullptr, nullptr));
}

// ---------------------------------------------------------------- keyframe prior

TEST(KeyframePrior, AcceptsOnlyTheUsefulBaselineWindow) {
    const Quat reference = scand3d::sensors::identityQuat();
    scand3d::sensors::KeyframePriorPolicy policy;  // 0.10 .. 0.70 rad
    const double deg = kPi / 180.0;

    EXPECT_FALSE(scand3d::sensors::acceptKeyframe(
        reference, quatFromAxisAngle({0.0, 0.0, 1.0}, 2.0 * deg), policy));
    EXPECT_TRUE(scand3d::sensors::acceptKeyframe(
        reference, quatFromAxisAngle({0.0, 0.0, 1.0}, 15.0 * deg), policy));
    EXPECT_TRUE(scand3d::sensors::acceptKeyframe(
        reference, quatFromAxisAngle({0.0, 0.0, 1.0}, 35.0 * deg), policy));
    EXPECT_FALSE(scand3d::sensors::acceptKeyframe(
        reference, quatFromAxisAngle({0.0, 0.0, 1.0}, 60.0 * deg), policy));
}

TEST(KeyframePrior, RejectsNonFiniteInput) {
    const Quat reference = scand3d::sensors::identityQuat();
    const Quat broken{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0};
    scand3d::sensors::KeyframePriorPolicy policy;
    EXPECT_FALSE(scand3d::sensors::acceptKeyframe(reference, broken, policy));
}

// ---------------------------------------------------------------- bridge handles

TEST(BridgeImu, ProducesAnOrientationFromAccelerometerAndGyro) {
    using namespace scand3d::bridge;
    const ImuHandle handle = imuCreate(200.0);
    ASSERT_NE(0u, handle);

    double values[kImuSampleValueCount] = {
        1'000'000'000.0, 0.0, 0.0, 0.0, 0.0, 0.0, kGravity, 0.0, 0.0, 0.0, 0.0,
    };
    values[10] = 3.0;  // gyro + accel

    ASSERT_TRUE(imuUpdate(handle, values, kImuSampleValueCount));
    for (int i = 1; i <= 50; ++i) {
        values[0] = static_cast<double>(1'000'000'000 + i * 5'000'000);
        ASSERT_TRUE(imuUpdate(handle, values, kImuSampleValueCount));
    }

    double quaternion[4] = {0.0, 0.0, 0.0, 0.0};
    ASSERT_TRUE(imuOrientation(handle, quaternion));
    const Quat q{quaternion[0], quaternion[1], quaternion[2], quaternion[3]};
    EXPECT_NEAR(0.0, quatAngleRad(scand3d::sensors::identityQuat(), q), 0.01);

    imuDestroy(handle);
    EXPECT_FALSE(imuOrientation(handle, quaternion));
}

TEST(BridgeImu, RunsTheFallbackWithoutAGyroscope) {
    using namespace scand3d::bridge;
    const ImuHandle handle = imuCreate(100.0);

    double values[kImuSampleValueCount] = {
        1'000'000'000.0, 0.0, 0.0, 0.0, -kGravity, 0.0, 0.0, 0.0, 40.0, 0.0, 0.0,
    };
    values[10] = 6.0;  // accel + mag, no gyro

    ASSERT_TRUE(imuUpdate(handle, values, kImuSampleValueCount));
    double quaternion[4] = {0.0, 0.0, 0.0, 0.0};
    ASSERT_TRUE(imuOrientation(handle, quaternion));
    EXPECT_TRUE(isFiniteQuat({quaternion[0], quaternion[1], quaternion[2], quaternion[3]}));

    imuDestroy(handle);
}

TEST(BridgeImu, RejectsUnknownHandlesAndMalformedSamples) {
    using namespace scand3d::bridge;
    double values[kImuSampleValueCount] = {
        1'000'000'000.0, 0.0, 0.0, 0.0, 0.0, 0.0, kGravity, 0.0, 0.0, 0.0, 3.0,
    };
    EXPECT_FALSE(imuUpdate(9999u, values, kImuSampleValueCount));
    EXPECT_FALSE(imuUpdate(9999u, values, kImuSampleValueCount - 1));

    const ImuHandle handle = imuCreate(200.0);
    ASSERT_FALSE(imuUpdate(handle, values, kImuSampleValueCount - 1));
    EXPECT_FALSE(imuUpdate(handle, nullptr, kImuSampleValueCount));

    values[0] = 0.0;  // not a clock value
    EXPECT_FALSE(imuUpdate(handle, values, kImuSampleValueCount));
    values[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(imuUpdate(handle, values, kImuSampleValueCount));

    // Resetting forgets the orientation until a usable sample arrives again.
    values[0] = 1'000'000'000.0;
    ASSERT_TRUE(imuUpdate(handle, values, kImuSampleValueCount));
    imuReset(handle);
    double quaternion[4];
    EXPECT_FALSE(imuOrientation(handle, quaternion));

    // Destroying twice must not fault.
    imuDestroy(handle);
    imuDestroy(handle);
}

TEST(BridgeImu, ComputesAngularDisplacement) {
    const Quat identity = scand3d::sensors::identityQuat();
    const Quat quarter = quatFromAxisAngle({0.0, 1.0, 0.0}, 0.5 * kPi);
    const double first[4] = {identity[0], identity[1], identity[2], identity[3]};
    const double second[4] = {quarter[0], quarter[1], quarter[2], quarter[3]};

    EXPECT_NEAR(0.5 * kPi, scand3d::bridge::angularDisplacementRad(first, second), 1e-9);
    EXPECT_TRUE(std::isnan(scand3d::bridge::angularDisplacementRad(nullptr, second)));

    double broken[4] = {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0};
    EXPECT_TRUE(std::isnan(scand3d::bridge::angularDisplacementRad(first, broken)));
}

TEST(BridgeImu, AppliesTheKeyframeWindow) {
    const Quat identity = scand3d::sensors::identityQuat();
    const Quat near = quatFromAxisAngle({0.0, 1.0, 0.0}, 2.0 * kPi / 180.0);
    const Quat good = quatFromAxisAngle({0.0, 1.0, 0.0}, 12.0 * kPi / 180.0);
    const double reference[4] = {identity[0], identity[1], identity[2], identity[3]};

    const double near_values[4] = {near[0], near[1], near[2], near[3]};
    const double good_values[4] = {good[0], good[1], good[2], good[3]};

    EXPECT_FALSE(scand3d::bridge::keyframeBaselineAccept(reference, near_values, 0.10, 0.70));
    EXPECT_TRUE(scand3d::bridge::keyframeBaselineAccept(reference, good_values, 0.10, 0.70));
    EXPECT_FALSE(scand3d::bridge::keyframeBaselineAccept(reference, nullptr, 0.10, 0.70));
}

}  // namespace
