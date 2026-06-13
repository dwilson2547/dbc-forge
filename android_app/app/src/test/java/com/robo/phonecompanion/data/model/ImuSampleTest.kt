package com.robo.phonecompanion.data.model

import org.junit.Assert.assertEquals
import org.junit.Test

/**
 * Validates the mount-tolerant yaw rate: the vertical-axis component recovered from the
 * fusion quaternion + gyro, so a dongle tilted in the OBD-II port still reads true
 * vehicle yaw rate.
 */
class ImuSampleTest {

    private fun sample(
        qw: Float, qx: Float, qy: Float, qz: Float,
        gx: Float, gy: Float, gz: Float,
    ) = ImuSample(0L, qw, qx, qy, qz, 0f, 0f, 0f, gx, gy, gz)

    @Test
    fun `flat mount reads yaw on sensor Z`() {
        // Identity orientation: world-up = sensor +Z, so yaw rate = gz.
        val s = sample(1f, 0f, 0f, 0f, gx = 0f, gy = 0f, gz = 0.5f)
        assertEquals(0.5f * 57.29578f, s.verticalYawRateDegPerSec, 1e-3f)
    }

    @Test
    fun `90 degree tilt about X moves yaw onto sensor Y`() {
        // Rotated 90° about X → world-up points along sensor +Y. A vehicle yaw then shows
        // up on the gyro Y axis, not Z; the projection must recover it.
        val r = 0.70710678f
        val s = sample(r, r, 0f, 0f, gx = 0f, gy = 1.0f, gz = 7.7f /* should be ignored */)
        assertEquals(1.0f * 57.29578f, s.verticalYawRateDegPerSec, 1e-2f)
    }
}
