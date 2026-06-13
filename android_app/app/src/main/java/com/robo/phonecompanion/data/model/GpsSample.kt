package com.robo.phonecompanion.data.model

import kotlinx.serialization.Serializable

/**
 * One GNSS fix used as a ground-truth reference channel for signal-discovery
 * correlation (Phase 11). [timestampMs] is wall-clock (`System.currentTimeMillis()`),
 * matching the timestamp domain of recorded CAN frames so the two can be aligned.
 *
 * [speedMps] is the receiver's Doppler-derived speed-over-ground — the primary
 * reference for `VEHICLE_SPEED` / `WHEEL_SPEED_*`. Only trust it when [hasSpeed].
 */
@Serializable
data class GpsSample(
    val timestampMs: Long,
    val latitude: Double,
    val longitude: Double,
    val speedMps: Float,
    val bearingDeg: Float,
    val accuracyM: Float,
    val hasSpeed: Boolean,
) {
    val speedKmh: Float get() = speedMps * 3.6f
}
