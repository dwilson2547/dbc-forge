package com.robo.phonecompanion.vm

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.robo.phonecompanion.data.decoder.SignalCorrelator
import com.robo.phonecompanion.data.decoder.SignalDecoder
import com.robo.phonecompanion.data.model.CanFrame
import com.robo.phonecompanion.data.model.DbcMessage
import com.robo.phonecompanion.data.model.GpsSample
import com.robo.phonecompanion.data.model.ImuSample
import com.robo.phonecompanion.data.model.SessionMeta
import com.robo.phonecompanion.data.repository.DbcRepository
import com.robo.phonecompanion.data.repository.SessionRepository
import com.robo.phonecompanion.vm.CanBusViewModel
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.flow.stateIn
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import java.io.File

data class SignalDiff(
    val signalName: String,
    val messageName: String,
    val valueAtA: Double?,
    val valueAtB: Double?,
) {
    val delta: Double? get() =
        if (valueAtA != null && valueAtB != null) valueAtB - valueAtA else null
}

data class SparklineSeries(
    val key: String,        // "$msgName/$sigName"
    val signalName: String,
    val messageName: String,
    val points: FloatArray, // normalized [0,1] Y values at evenly-spaced time slots
)

/** Phase 11 — a ground-truth reference channel the correlator can match CAN bits against. */
enum class ReferenceChannel(val label: String, val unit: String, val targetHint: String) {
    GPS_SPEED("GPS speed", "km/h", "VEHICLE_SPEED / WHEEL_SPEED_*"),
    IMU_YAW_RATE("IMU yaw rate", "deg/s", "YAW_RATE / STEERING_ANGLE"),
    // Derived kinematically (no IMU-accel orientation calibration needed):
    LONG_ACCEL("Long accel", "m/s²", "LONGITUDINAL_ACCEL / brake / throttle"),
    LAT_ACCEL("Lateral accel", "m/s²", "LATERAL_ACCEL"),
}

/** One candidate bit field that correlated with a channel, tagged with its CAN ID. */
data class ScanHit(
    val canId: Int,
    val isExtended: Boolean,
    val channel: ReferenceChannel,
    val candidate: SignalCorrelator.Candidate,
    val knownMessage: String? = null,   // message name if this ID is already in the DBC
    val overlapsSignal: String? = null, // existing signal this bit window re-detects, if any
    val dlc: Int = 8,                   // frame length for this ID (for prefilling a new message)
)

/** Result of a discovery run — either one CAN ID or a full sweep of unknown IDs. */
data class DiscoveryRun(
    val scopeLabel: String,        // "0x1A0" or "All unknown IDs"
    val channelsUsed: List<ReferenceChannel>,
    val hits: List<ScanHit>,       // ranked by |r| desc
    val idsScanned: Int,
    val note: String = "",         // explanation when hits is empty
)

class LogPlayerViewModel(application: Application) : AndroidViewModel(application) {

    private val sessionRepository = SessionRepository(File(application.filesDir, "sessions"))
    private val dbcRepository = DbcRepository(File(application.filesDir, "git_repo/dbcs"))

    private val _isLoading = MutableStateFlow(false)
    val isLoading: StateFlow<Boolean> = _isLoading.asStateFlow()

    private val _meta = MutableStateFlow<SessionMeta?>(null)
    val meta: StateFlow<SessionMeta?> = _meta.asStateFlow()

    private val _allFrames = MutableStateFlow<List<DisplayFrame>>(emptyList())
    val allFrames: StateFlow<List<DisplayFrame>> = _allFrames.asStateFlow()

    // Phase 11 — reference channels for this session (empty if none recorded).
    private val _gpsSamples = MutableStateFlow<List<GpsSample>>(emptyList())
    val gpsSamples: StateFlow<List<GpsSample>> = _gpsSamples.asStateFlow()

    private val _imuSamples = MutableStateFlow<List<ImuSample>>(emptyList())
    val imuSamples: StateFlow<List<ImuSample>> = _imuSamples.asStateFlow()

    private val _discovery = MutableStateFlow<DiscoveryRun?>(null)
    val discovery: StateFlow<DiscoveryRun?> = _discovery.asStateFlow()

    private val _discoveryRunning = MutableStateFlow(false)
    val discoveryRunning: StateFlow<Boolean> = _discoveryRunning.asStateFlow()

    // Progress for a full sweep: ids done / total (0/0 when idle or single-ID).
    private val _discoveryProgress = MutableStateFlow(0 to 0)
    val discoveryProgress: StateFlow<Pair<Int, Int>> = _discoveryProgress.asStateFlow()

    private val _showKnown = MutableStateFlow(true)
    val showKnown: StateFlow<Boolean> = _showKnown.asStateFlow()

    private val _showUnknown = MutableStateFlow(true)
    val showUnknown: StateFlow<Boolean> = _showUnknown.asStateFlow()

    private val _showDiag = MutableStateFlow(true)
    val showDiag: StateFlow<Boolean> = _showDiag.asStateFlow()

    // Phase 6.1 — Timeline range scrubber (normalised [0, 1])
    private val _rangeStart = MutableStateFlow(0f)
    val rangeStart: StateFlow<Float> = _rangeStart.asStateFlow()

    private val _rangeEnd = MutableStateFlow(1f)
    val rangeEnd: StateFlow<Float> = _rangeEnd.asStateFlow()

    // Phase 6.2 — Sparklines (one per decoded signal, 200-slot time series)
    private val _sparklines = MutableStateFlow<List<SparklineSeries>>(emptyList())
    val sparklines: StateFlow<List<SparklineSeries>> = _sparklines.asStateFlow()

    // Phase 6.3 — Reference event anchor
    private val _referenceTs = MutableStateFlow<Long?>(null)
    val referenceTs: StateFlow<Long?> = _referenceTs.asStateFlow()

    // Phase 6.4 — Stimulus-response diff
    private val _bookmarkA = MutableStateFlow<Long?>(null)
    val bookmarkA: StateFlow<Long?> = _bookmarkA.asStateFlow()

    private val _bookmarkB = MutableStateFlow<Long?>(null)
    val bookmarkB: StateFlow<Long?> = _bookmarkB.asStateFlow()

    private val _diffResult = MutableStateFlow<List<SignalDiff>>(emptyList())
    val diffResult: StateFlow<List<SignalDiff>> = _diffResult.asStateFlow()

    // Phase 6.5 — Multi-signal correlation: up to 4 selected signal keys
    private val _selectedSignalKeys = MutableStateFlow<List<String>>(emptyList())
    val selectedSignalKeys: StateFlow<List<String>> = _selectedSignalKeys.asStateFlow()

    // Phase 6.6 — Playback
    private val _isPlaying = MutableStateFlow(false)
    val isPlaying: StateFlow<Boolean> = _isPlaying.asStateFlow()

    private val _playbackSpeed = MutableStateFlow(1f)
    val playbackSpeed: StateFlow<Float> = _playbackSpeed.asStateFlow()

    private var playbackJob: Job? = null

    // visibleFrames: range filter → known/unknown/diag filter
    val visibleFrames: StateFlow<List<DisplayFrame>> =
        combine(
            combine(_allFrames, _rangeStart, _rangeEnd) { frames, start, end ->
                if (frames.isEmpty()) emptyList()
                else {
                    val minTs = frames.first().frame.timestampMs
                    val maxTs = frames.last().frame.timestampMs
                    val dur = (maxTs - minTs).coerceAtLeast(1L)
                    val tStart = minTs + (dur * start).toLong()
                    val tEnd = minTs + (dur * end).toLong()
                    frames.filter { df -> df.frame.timestampMs in tStart..tEnd }
                }
            },
            _showKnown,
            _showUnknown,
            _showDiag,
        ) { rangedFrames, known, unknown, diag ->
            rangedFrames.filter { df ->
                val isDiag = CanBusViewModel.isObd2Diagnostic(df.frame.id)
                when {
                    isDiag -> diag
                    df.message != null -> known
                    else -> unknown
                }
            }
        }.stateIn(viewModelScope, SharingStarted.Eagerly, emptyList())

    // ── Public API ────────────────────────────────────────────────────────────

    fun toggleShowKnown() { _showKnown.value = !_showKnown.value }
    fun toggleShowUnknown() { _showUnknown.value = !_showUnknown.value }
    fun toggleShowDiag() { _showDiag.value = !_showDiag.value }

    fun setRange(start: Float, end: Float) {
        _rangeStart.value = start.coerceIn(0f, 1f)
        _rangeEnd.value = end.coerceIn(0f, 1f)
    }

    fun setReference(ts: Long) { _referenceTs.value = ts }
    fun clearReference() { _referenceTs.value = null }

    fun setBookmarkA(ts: Long) { _bookmarkA.value = ts; recomputeDiff() }
    fun setBookmarkB(ts: Long) { _bookmarkB.value = ts; recomputeDiff() }
    fun clearBookmarkA() { _bookmarkA.value = null; _diffResult.value = emptyList() }
    fun clearBookmarkB() { _bookmarkB.value = null; _diffResult.value = emptyList() }
    fun clearDiff() { _bookmarkA.value = null; _bookmarkB.value = null; _diffResult.value = emptyList() }

    fun toggleSignalSelection(key: String) {
        val current = _selectedSignalKeys.value
        _selectedSignalKeys.value = if (current.contains(key)) current - key
        else if (current.size < MAX_CORRELATION_SIGNALS) current + key
        else current
    }

    fun clearSignalSelection() { _selectedSignalKeys.value = emptyList() }

    fun startPlayback() {
        val frames = _allFrames.value
        if (frames.size < 2) return
        val sessionMinTs = frames.first().frame.timestampMs
        val sessionDur = (frames.last().frame.timestampMs - sessionMinTs).coerceAtLeast(1L).toFloat()

        stopPlayback()
        // Begin cursor at current rangeEnd; if at the end, restart from rangeStart
        var cursorMs = if (_rangeEnd.value >= 0.999f) {
            sessionMinTs + (sessionDur * _rangeStart.value).toLong()
        } else {
            sessionMinTs + (sessionDur * _rangeEnd.value).toLong()
        }

        _isPlaying.value = true
        playbackJob = viewModelScope.launch {
            var lastWallMs = System.currentTimeMillis()
            while (isActive) {
                delay(16)
                val now = System.currentTimeMillis()
                val wallElapsed = now - lastWallMs
                lastWallMs = now
                cursorMs += (wallElapsed * _playbackSpeed.value).toLong()
                val newPos = ((cursorMs - sessionMinTs) / sessionDur).coerceIn(0f, 1f)
                _rangeEnd.value = newPos
                if (newPos >= 1f) {
                    _isPlaying.value = false
                    break
                }
            }
        }
    }

    fun stopPlayback() {
        playbackJob?.cancel()
        playbackJob = null
        _isPlaying.value = false
    }

    fun setPlaybackSpeed(speed: Float) { _playbackSpeed.value = speed }

    fun loadSession(sessionId: String) {
        if (_meta.value?.id == sessionId) return
        stopPlayback()
        viewModelScope.launch(Dispatchers.IO) {
            _isLoading.value = true
            _allFrames.value = emptyList()
            _gpsSamples.value = emptyList()
            _imuSamples.value = emptyList()
            _discovery.value = null
            _sparklines.value = emptyList()
            _selectedSignalKeys.value = emptyList()
            _rangeStart.value = 0f
            _rangeEnd.value = 1f
            _showKnown.value = true
            _showUnknown.value = true
            _showDiag.value = true
            _referenceTs.value = null
            _bookmarkA.value = null
            _bookmarkB.value = null
            _diffResult.value = emptyList()

            val meta = sessionRepository.loadMeta(sessionId) ?: run {
                _isLoading.value = false
                return@launch
            }
            _meta.value = meta

            val dbc = if (meta.dbcId != "none")
                runCatching { dbcRepository.load(meta.dbcId) }.getOrNull()
            else null
            val framesFile = sessionRepository.framesFile(sessionId)
            if (!framesFile.exists()) { _isLoading.value = false; return@launch }

            val result = mutableListOf<DisplayFrame>()
            var seq = 0L

            framesFile.forEachLine { line ->
                val parts = line.split(",")
                if (parts.size < 5) return@forEachLine
                val ts = parts[0].toLongOrNull() ?: return@forEachLine
                val id = parts[1].removePrefix("0x").toIntOrNull(16) ?: return@forEachLine
                val ext = parts[2] == "1"
                val dlc = parts[3].toIntOrNull() ?: return@forEachLine
                val hex = parts[4].trimEnd()
                val data = if (hex.length >= dlc * 2) {
                    ByteArray(dlc) { i -> hex.substring(i * 2, i * 2 + 2).toInt(16).toByte() }
                } else ByteArray(dlc)

                val frame = CanFrame(timestampMs = ts, id = id, isExtended = ext, data = data)
                val message = dbc?.messageForCanId(id)
                val signals: Map<String, Double> = message?.signals
                    ?.mapNotNull { sig -> SignalDecoder.decodeOrNull(sig, data)?.let { sig.name to it } }
                    ?.toMap()
                    ?: emptyMap()

                result.add(DisplayFrame(frame = frame, message = message, decodedSignals = signals, seq = seq++))
            }

            val gpsFile = sessionRepository.gpsFile(sessionId)
            val gps = if (gpsFile.exists()) parseGps(gpsFile) else emptyList()
            val imuFile = sessionRepository.imuFile(sessionId)
            val imu = if (imuFile.exists()) parseImu(imuFile) else emptyList()

            _allFrames.value = result
            _gpsSamples.value = gps
            _imuSamples.value = imu
            _sparklines.value = computeSparklines(result)
            _isLoading.value = false
        }
    }

    // IMU line format: timestampMs,qw,qx,qy,qz,ax,ay,az,gx,gy,gz
    private fun parseImu(file: File): List<ImuSample> {
        val out = mutableListOf<ImuSample>()
        file.forEachLine { line ->
            val p = line.split(",")
            if (p.size < 11) return@forEachLine
            val ts = p[0].toLongOrNull() ?: return@forEachLine
            fun f(i: Int) = p[i].toFloatOrNull() ?: 0f
            out.add(
                ImuSample(
                    timestampMs = ts,
                    qw = f(1), qx = f(2), qy = f(3), qz = f(4),
                    ax = f(5), ay = f(6), az = f(7),
                    gx = f(8), gy = f(9), gz = f(10),
                )
            )
        }
        return out
    }

    // GPS line format: timestampMs,lat,lon,speedMps,bearingDeg,accuracyM,hasSpeed
    private fun parseGps(file: File): List<GpsSample> {
        val out = mutableListOf<GpsSample>()
        file.forEachLine { line ->
            val p = line.split(",")
            if (p.size < 7) return@forEachLine
            val ts = p[0].toLongOrNull() ?: return@forEachLine
            out.add(
                GpsSample(
                    timestampMs = ts,
                    latitude = p[1].toDoubleOrNull() ?: 0.0,
                    longitude = p[2].toDoubleOrNull() ?: 0.0,
                    speedMps = p[3].toFloatOrNull() ?: 0f,
                    bearingDeg = p[4].toFloatOrNull() ?: 0f,
                    accuracyM = p[5].toFloatOrNull() ?: 0f,
                    hasSpeed = p[6].trim() == "1",
                )
            )
        }
        return out
    }

    // ── Phase 11: brute-force correlation against reference channels ─────────────

    fun clearDiscovery() { _discovery.value = null }

    /** Reference channels that actually have usable (varying) data this session. */
    fun availableChannels(): List<ReferenceChannel> = buildList {
        val hasGps = _gpsSamples.value.any { it.hasSpeed }
        val hasImu = _imuSamples.value.isNotEmpty()
        if (hasGps) add(ReferenceChannel.GPS_SPEED)
        if (hasImu) add(ReferenceChannel.IMU_YAW_RATE)
        if (hasGps) add(ReferenceChannel.LONG_ACCEL)         // d(speed)/dt
        if (hasGps && hasImu) add(ReferenceChannel.LAT_ACCEL) // v · yawRate
    }

    /** Linear interpolation of GPS speed (m/s) at time [t]; assumes [gps] sorted by time. */
    private fun interpSpeedMps(gps: List<GpsSample>, t: Long): Double {
        if (gps.isEmpty()) return 0.0
        if (t <= gps.first().timestampMs) return gps.first().speedMps.toDouble()
        if (t >= gps.last().timestampMs) return gps.last().speedMps.toDouble()
        var lo = 0; var hi = gps.size - 1
        while (hi - lo > 1) {
            val mid = (lo + hi) / 2
            if (gps[mid].timestampMs <= t) lo = mid else hi = mid
        }
        val a = gps[lo]; val b = gps[hi]
        if (b.timestampMs == a.timestampMs) return a.speedMps.toDouble()
        val frac = (t - a.timestampMs).toDouble() / (b.timestampMs - a.timestampMs)
        return a.speedMps + frac * (b.speedMps - a.speedMps)
    }

    private fun refPointsFor(channel: ReferenceChannel): List<SignalCorrelator.RefPoint> =
        when (channel) {
            ReferenceChannel.GPS_SPEED -> _gpsSamples.value
                .filter { it.hasSpeed }
                .map { SignalCorrelator.RefPoint(it.timestampMs, it.speedKmh.toDouble()) }

            // Longitudinal accel = d(speed)/dt from consecutive GPS fixes (m/s²).
            ReferenceChannel.LONG_ACCEL -> {
                val gps = _gpsSamples.value.filter { it.hasSpeed }.sortedBy { it.timestampMs }
                buildList {
                    for (i in 1 until gps.size) {
                        val dt = (gps[i].timestampMs - gps[i - 1].timestampMs) / 1000.0
                        if (dt > 0.05) {
                            val a = (gps[i].speedMps - gps[i - 1].speedMps) / dt
                            val t = (gps[i].timestampMs + gps[i - 1].timestampMs) / 2
                            add(SignalCorrelator.RefPoint(t, a.toDouble()))
                        }
                    }
                }
            }

            // Lateral accel = v · yawRate (centripetal), at each 50 Hz IMU sample (m/s²).
            ReferenceChannel.LAT_ACCEL -> {
                val gps = _gpsSamples.value.filter { it.hasSpeed }.sortedBy { it.timestampMs }
                _imuSamples.value.map { imu ->
                    val v = interpSpeedMps(gps, imu.timestampMs)
                    val omega = imu.verticalYawRateDegPerSec / 57.29578   // rad/s
                    SignalCorrelator.RefPoint(imu.timestampMs, v * omega)
                }
            }

            ReferenceChannel.IMU_YAW_RATE -> _imuSamples.value
                .map { SignalCorrelator.RefPoint(it.timestampMs, it.verticalYawRateDegPerSec.toDouble()) }
        }

    private fun messageForId(canId: Int): DbcMessage? =
        _allFrames.value.firstOrNull { it.frame.id == canId }?.message

    /** Discovery for a single CAN [canId] across every available channel. */
    fun runIdDiscovery(canId: Int, isExtended: Boolean) {
        if (_discoveryRunning.value) return
        val channels = availableChannels()
        val idLabel = if (isExtended) "0x%08X".format(canId) else "0x%03X".format(canId)
        val message = messageForId(canId)
        viewModelScope.launch(Dispatchers.Default) {
            _discoveryRunning.value = true
            _discoveryProgress.value = 0 to 0
            try {
                val hits = mutableListOf<ScanHit>()
                // Explicit ID: show everything, including bits that overlap existing
                // signals (so a misplaced definition can be corrected against ground truth).
                channels.forEach { ch ->
                    hits += scanId(canId, isExtended, ch, maxResults = 6, message, excludeDefined = false)
                }
                hits.sortByDescending { kotlin.math.abs(it.candidate.correlation) }
                _discovery.value = DiscoveryRun(
                    scopeLabel = idLabel,
                    channelsUsed = channels,
                    hits = hits,
                    idsScanned = 1,
                    note = discoveryNote(channels, hits.isEmpty()),
                )
            } finally {
                _discoveryRunning.value = false
            }
        }
    }

    /**
     * Discovery across **all** CAN IDs against every available channel. Unknown IDs report
     * every candidate; known (DBC-defined) IDs report only candidates in *undecoded* bits —
     * so new signals in partially-mapped messages surface without re-reporting what's already
     * defined.
     */
    fun runFullDiscovery() {
        if (_discoveryRunning.value) return
        val channels = availableChannels()
        viewModelScope.launch(Dispatchers.Default) {
            _discoveryRunning.value = true
            try {
                val ids = _allFrames.value.distinctBy { it.frame.id }
                    .map { Triple(it.frame.id, it.frame.isExtended, it.message) }
                _discoveryProgress.value = 0 to ids.size
                val hits = mutableListOf<ScanHit>()
                ids.forEachIndexed { i, (id, ext, message) ->
                    channels.forEach { ch ->
                        hits += scanId(id, ext, ch, maxResults = 3, message, excludeDefined = true)
                    }
                    _discoveryProgress.value = (i + 1) to ids.size
                }
                hits.sortByDescending { kotlin.math.abs(it.candidate.correlation) }
                _discovery.value = DiscoveryRun(
                    scopeLabel = "All IDs",
                    channelsUsed = channels,
                    hits = hits,
                    idsScanned = ids.size,
                    note = discoveryNote(channels, hits.isEmpty()),
                )
            } finally {
                _discoveryRunning.value = false
            }
        }
    }

    /** Correlate one CAN ID's frames against one channel; returns tagged hits. */
    private fun scanId(
        canId: Int,
        isExtended: Boolean,
        channel: ReferenceChannel,
        maxResults: Int,
        message: DbcMessage?,
        excludeDefined: Boolean,
    ): List<ScanHit> {
        val idFrames = _allFrames.value.filter { it.frame.id == canId }
        if (idFrames.size < 5) return emptyList()
        val dlc = idFrames.maxOfOrNull { it.frame.data.size } ?: 0
        val framePoints = idFrames.map {
            SignalCorrelator.FramePoint(it.frame.timestampMs, it.frame.data)
        }
        val result = SignalCorrelator.correlate(
            framePoints, refPointsFor(channel), dlc, maxResults = maxResults,
        )
        if (result.status != SignalCorrelator.Status.OK) return emptyList()

        var candidates = result.candidates
        if (excludeDefined && message != null && message.signals.isNotEmpty()) {
            val defined = message.signals
                .flatMap { SignalCorrelator.bitsCovered(it.startBit, it.length, it.byteOrder) }
                .toSet()
            candidates = candidates.filter { c ->
                val cb = SignalCorrelator.bitsCovered(c.startBit, c.length, c.byteOrder)
                cb.count { it in defined } <= cb.size / 2   // keep only mostly-undecoded windows
            }
        }
        return candidates.map { c ->
            val cb = SignalCorrelator.bitsCovered(c.startBit, c.length, c.byteOrder)
            // Which existing signal (if any) does this window re-detect? Tag it so the user
            // can tell new signals from already-mapped ones.
            val overlap = message?.signals
                ?.map { it.name to SignalCorrelator.bitsCovered(it.startBit, it.length, it.byteOrder).count { b -> b in cb } }
                ?.maxByOrNull { it.second }
            val overlapName = overlap?.takeIf { it.second * 4 >= cb.size }?.first  // ≥25% of candidate bits
            ScanHit(canId, isExtended, channel, c, knownMessage = message?.name,
                overlapsSignal = overlapName, dlc = dlc)
        }
    }

    private fun discoveryNote(channels: List<ReferenceChannel>, empty: Boolean): String = when {
        channels.isEmpty() ->
            "No reference channels recorded — this session has no GPS speed or IMU data."
        empty ->
            "No bit fields correlated above threshold. For yaw rate you need recorded turns; " +
                "for speed, recorded acceleration/braking."
        else -> ""
    }

    // ── Diff helpers ──────────────────────────────────────────────────────────

    private fun recomputeDiff() {
        viewModelScope.launch(Dispatchers.Default) {
            val tsA = _bookmarkA.value ?: return@launch
            val tsB = _bookmarkB.value ?: return@launch
            _diffResult.value = computeDiff(_allFrames.value, tsA, tsB)
        }
    }

    private fun buildSnapshot(frames: List<DisplayFrame>, targetTs: Long): Map<String, Double> {
        val result = mutableMapOf<String, Double>()
        frames.filter { it.frame.timestampMs in (targetTs - SNAPSHOT_WINDOW_MS)..targetTs }
            .forEach { df ->
                val msgName = df.message?.name ?: return@forEach
                df.decodedSignals.forEach { (sigName, value) ->
                    result["$msgName/$sigName"] = value
                }
            }
        return result
    }

    private fun computeDiff(frames: List<DisplayFrame>, tsA: Long, tsB: Long): List<SignalDiff> {
        val (earlier, later) = if (tsA <= tsB) tsA to tsB else tsB to tsA
        val snapshotA = buildSnapshot(frames, earlier)
        val snapshotB = buildSnapshot(frames, later)
        val allKeys = (snapshotA.keys + snapshotB.keys).toSet()
        return allKeys.map { key ->
            val parts = key.split("/", limit = 2)
            SignalDiff(
                signalName = parts.getOrElse(1) { key },
                messageName = parts.getOrElse(0) { "?" },
                valueAtA = snapshotA[key],
                valueAtB = snapshotB[key],
            )
        }.sortedByDescending { diff ->
            diff.delta?.let { kotlin.math.abs(it) } ?: 0.0
        }
    }

    // ── Sparkline computation ─────────────────────────────────────────────────

    private fun computeSparklines(frames: List<DisplayFrame>): List<SparklineSeries> {
        if (frames.size < 2) return emptyList()
        val minTs = frames.first().frame.timestampMs
        val maxTs = frames.last().frame.timestampMs
        val dur = maxTs - minTs
        if (dur <= 0L) return emptyList()

        // Collect time-ordered (ts, value) pairs per signal key
        val rawMap = mutableMapOf<String, MutableList<Pair<Long, Double>>>()
        frames.forEach { df ->
            val msgName = df.message?.name ?: return@forEach
            df.decodedSignals.forEach { (sigName, value) ->
                rawMap.getOrPut("$msgName/$sigName") { mutableListOf() }
                    .add(df.frame.timestampMs to value)
            }
        }

        return rawMap.map { (key, values) ->
            val sampled = FloatArray(SPARKLINE_SLOTS)
            var vi = 0
            for (slot in 0 until SPARKLINE_SLOTS) {
                val targetTs = minTs + dur * slot / (SPARKLINE_SLOTS - 1)
                while (vi + 1 < values.size && values[vi + 1].first <= targetTs) vi++
                sampled[slot] = values.getOrNull(vi)?.second?.toFloat() ?: 0f
            }
            // Normalize to [0, 1]
            val sMin = sampled.min()
            val sMax = sampled.max()
            val sRange = sMax - sMin
            if (sRange > 1e-6f) {
                for (i in sampled.indices) sampled[i] = (sampled[i] - sMin) / sRange
            } else {
                sampled.fill(0.5f)
            }
            val parts = key.split("/", limit = 2)
            SparklineSeries(
                key = key,
                signalName = parts.getOrElse(1) { key },
                messageName = parts.getOrElse(0) { "?" },
                points = sampled,
            )
        }.sortedWith(compareBy({ it.messageName }, { it.signalName }))
    }

    companion object {
        const val SNAPSHOT_WINDOW_MS = 2_000L
        const val MAX_CORRELATION_SIGNALS = 4
        private const val SPARKLINE_SLOTS = 200
    }
}
