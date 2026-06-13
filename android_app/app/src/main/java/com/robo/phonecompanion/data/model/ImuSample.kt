package com.robo.phonecompanion.data.model

/**
 * One fused IMU sample from the dongle's BNO085 (Phase 11.3), streamed over BLE as an
 * `IMU,...` NUS line and timestamped on the dongle's shared `millis()` clock — the same
 * domain as CAN frames, so the two align without a cross-clock search.
 *
 * Quaternion is the absolute rotation vector (sensor→world). [ax]/[ay]/[az] are
 * gravity-removed linear acceleration (m/s²). [gx]/[gy]/[gz] are calibrated angular
 * rate (rad/s); [gz] is the sensor-frame yaw rate. Lateral/longitudinal mapping needs
 * the mount orientation applied first (Phase 11.6).
 */
data class ImuSample(
    val timestampMs: Long,
    val qw: Float,
    val qx: Float,
    val qy: Float,
    val qz: Float,
    val ax: Float,
    val ay: Float,
    val az: Float,
    val gx: Float,
    val gy: Float,
    val gz: Float,
    val firmwareTimestampMs: Long? = null,
) {
    /** Sensor-frame yaw rate in deg/s (raw gyro Z). */
    val yawRateDegPerSec: Float get() = gz * 57.29578f

    /**
     * Mount-tolerant yaw rate (deg/s): the angular-velocity component about **true
     * vertical**, found by projecting the gyro vector onto the gravity direction taken
     * from the fusion quaternion. This makes the channel independent of how the dongle is
     * tilted in the OBD-II port — no calibration step needed. Sign is arbitrary (the
     * correlator handles sign via signed r / regression).
     *
     * World-up [0,0,1] expressed in the sensor frame is the third row of the body→world
     * rotation matrix; the dot product with (gx,gy,gz) is the vertical-axis rate.
     */
    val verticalYawRateDegPerSec: Float
        get() {
            val ux = 2f * (qx * qz - qw * qy)
            val uy = 2f * (qy * qz + qw * qx)
            val uz = 1f - 2f * (qx * qx + qy * qy)
            return (gx * ux + gy * uy + gz * uz) * 57.29578f
        }
}
