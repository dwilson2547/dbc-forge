package com.robo.phonecompanion.data.sensor

import android.annotation.SuppressLint
import android.content.Context
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Bundle
import android.os.Looper
import com.robo.phonecompanion.data.model.GpsSample
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow

/**
 * Phone GNSS reference source (Phase 11.1). Uses the framework [LocationManager]
 * `GPS_PROVIDER` directly — no Google Play Services dependency, so it works on
 * non-GMS devices too. [Location.getSpeed] gives the chip's Doppler speed-over-ground,
 * which is the ground truth we correlate against.
 *
 * Callers must hold `ACCESS_FINE_LOCATION` before calling [start].
 */
class GpsSource(context: Context) {

    private val locationManager =
        context.getSystemService(Context.LOCATION_SERVICE) as LocationManager

    private val _latest = MutableStateFlow<GpsSample?>(null)
    val latest: StateFlow<GpsSample?> = _latest.asStateFlow()

    private val _samples = MutableSharedFlow<GpsSample>(extraBufferCapacity = 64)
    val samples: SharedFlow<GpsSample> = _samples.asSharedFlow()

    @Volatile private var running = false

    // Full interface (not a SAM lambda): onStatusChanged/onProvider* are abstract on
    // API 26–29 framework builds, so a lambda would AbstractMethodError at runtime.
    private val listener = object : LocationListener {
        override fun onLocationChanged(location: Location) = onLocation(location)
        @Deprecated("Deprecated in Java")
        override fun onStatusChanged(provider: String?, status: Int, extras: Bundle?) {}
        override fun onProviderEnabled(provider: String) {}
        override fun onProviderDisabled(provider: String) {}
    }

    fun isAvailable(): Boolean =
        runCatching { locationManager.allProviders.contains(LocationManager.GPS_PROVIDER) }
            .getOrDefault(false)

    /** Returns true if updates started. Safe to call when already running. */
    @SuppressLint("MissingPermission")
    fun start(): Boolean {
        if (running) return true
        if (!isAvailable()) return false
        return try {
            locationManager.requestLocationUpdates(
                LocationManager.GPS_PROVIDER,
                0L,   // minTime — emit as fast as the provider produces fixes (~1 Hz)
                0f,   // minDistance
                listener,
                Looper.getMainLooper(),
            )
            running = true
            true
        } catch (_: SecurityException) {
            false
        }
    }

    fun stop() {
        if (!running) return
        running = false
        runCatching { locationManager.removeUpdates(listener) }
    }

    private fun onLocation(location: Location) {
        val sample = GpsSample(
            timestampMs = System.currentTimeMillis(),
            latitude = location.latitude,
            longitude = location.longitude,
            speedMps = if (location.hasSpeed()) location.speed else 0f,
            bearingDeg = if (location.hasBearing()) location.bearing else 0f,
            accuracyM = if (location.hasAccuracy()) location.accuracy else 0f,
            hasSpeed = location.hasSpeed(),
        )
        _latest.value = sample
        _samples.tryEmit(sample)
    }
}
