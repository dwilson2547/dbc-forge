package com.robo.phonecompanion.data.repository

import com.robo.phonecompanion.data.model.CanFrame
import com.robo.phonecompanion.data.model.GpsSample
import com.robo.phonecompanion.data.model.ImuSample
import com.robo.phonecompanion.data.model.SessionMeta
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json
import java.io.File
import java.io.FileWriter

class SessionRepository(private val sessionsDir: File) {

    private val json = Json { prettyPrint = true; ignoreUnknownKeys = true }

    fun listAll(): List<SessionMeta> {
        if (!sessionsDir.exists()) return emptyList()
        return sessionsDir.listFiles { f -> f.isDirectory }
            ?.mapNotNull { dir ->
                runCatching {
                    json.decodeFromString<SessionMeta>(File(dir, "meta.json").readText())
                }.getOrNull()
            }
            ?.sortedByDescending { it.startTime }
            ?: emptyList()
    }

    fun createSession(meta: SessionMeta): ActiveSession {
        val dir = File(sessionsDir, meta.id)
        dir.mkdirs()
        File(dir, "meta.json").writeText(json.encodeToString(meta))
        return ActiveSession(dir, meta)
    }

    fun loadMeta(sessionId: String): SessionMeta? {
        val file = File(sessionsDir, "$sessionId/meta.json")
        if (!file.exists()) return null
        return runCatching { json.decodeFromString<SessionMeta>(file.readText()) }.getOrNull()
    }

    fun updateMeta(meta: SessionMeta) {
        val file = File(sessionsDir, "${meta.id}/meta.json")
        if (file.exists()) file.writeText(json.encodeToString(meta))
    }

    fun framesFile(sessionId: String) = File(sessionsDir, "$sessionId/frames.log")

    fun gpsFile(sessionId: String) = File(sessionsDir, "$sessionId/gps.log")

    fun imuFile(sessionId: String) = File(sessionsDir, "$sessionId/imu.log")

    inner class ActiveSession(private val dir: File, initialMeta: SessionMeta) {
        var meta = initialMeta
            private set

        private val writer = FileWriter(File(dir, "frames.log"), true)
        private var frameCount = 0

        // GPS sidecar is created lazily — sessions without a fix never spawn the file.
        private var gpsWriter: FileWriter? = null
        private var gpsCount = 0

        // IMU sidecar — also lazy (only when a BNO085-equipped dongle is connected).
        private var imuWriter: FileWriter? = null
        private var imuCount = 0

        // Frame line format: timestampMs,id,ext,dlc,hex_bytes
        fun appendFrame(frame: CanFrame) {
            try {
                val hex = frame.data.joinToString("") { "%02X".format(it) }
                val ext = if (frame.isExtended) "1" else "0"
                writer.write("${frame.timestampMs},0x%08X,$ext,${frame.data.size},$hex\n".format(frame.id))
                frameCount++
            } catch (_: Exception) { }
        }

        // GPS line format: timestampMs,lat,lon,speedMps,bearingDeg,accuracyM,hasSpeed
        fun appendGps(s: GpsSample) {
            try {
                val w = gpsWriter ?: FileWriter(File(dir, "gps.log"), true).also { gpsWriter = it }
                w.write("${s.timestampMs},${s.latitude},${s.longitude},${s.speedMps}," +
                    "${s.bearingDeg},${s.accuracyM},${if (s.hasSpeed) 1 else 0}\n")
                gpsCount++
            } catch (_: Exception) { }
        }

        // IMU line format: timestampMs,qw,qx,qy,qz,ax,ay,az,gx,gy,gz
        fun appendImu(s: ImuSample) {
            try {
                val w = imuWriter ?: FileWriter(File(dir, "imu.log"), true).also { imuWriter = it }
                w.write("${s.timestampMs},${s.qw},${s.qx},${s.qy},${s.qz}," +
                    "${s.ax},${s.ay},${s.az},${s.gx},${s.gy},${s.gz}\n")
                imuCount++
            } catch (_: Exception) { }
        }

        fun close(endTime: String) {
            writer.flush()
            writer.close()
            gpsWriter?.let { runCatching { it.flush(); it.close() } }
            imuWriter?.let { runCatching { it.flush(); it.close() } }
            meta = meta.copy(
                endTime = endTime, frameCount = frameCount,
                gpsSampleCount = gpsCount, imuSampleCount = imuCount,
            )
            File(dir, "meta.json").writeText(json.encodeToString(meta))
        }
    }
}
