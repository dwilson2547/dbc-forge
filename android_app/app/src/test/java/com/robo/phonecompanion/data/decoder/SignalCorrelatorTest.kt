package com.robo.phonecompanion.data.decoder

import com.robo.phonecompanion.data.model.ByteOrder
import com.robo.phonecompanion.data.model.DbcSignal
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI
import kotlin.math.roundToInt
import kotlin.math.sin

/**
 * Synthetic end-to-end validation of the Phase 11 correlator. We fabricate a "drive":
 * a known speed profile is encoded into a 16-bit field at the **start** of the frame
 * (bytes 0–1), with high-entropy noise in every other byte. Placing the field at
 * startBit 0 leaves no low-order bits to "shift down" into, and the noise elsewhere
 * spoils any window that reaches up past the field — so the genuine bit field is the
 * unique clean linear decode of speed. A matching GPS speed channel is sampled from the
 * same profile.
 *
 * The correlator must recover that field, a scale that decodes back to true speed, and
 * (when the reference is skewed) the time lag.
 */
class SignalCorrelatorTest {

    private val t0 = 1_000_000L
    private val factor = 0.01   // physical km/h = raw * 0.01

    private fun put16(data: ByteArray, startByte: Int, v: Int) {
        data[startByte] = (v and 0xFF).toByte()
        data[startByte + 1] = ((v shr 8) and 0xFF).toByte()
    }

    /** Frame with speed at bytes 0–1 and deterministic per-frame noise in bytes 2–7. */
    private fun frameFor(i: Int, t: Long, speed: Double): SignalCorrelator.FramePoint {
        val raw = (speed / factor).roundToInt().coerceIn(0, 65535)
        val data = ByteArray(8)
        put16(data, 0, raw)
        val rng = java.util.Random(i * 0x9E3779B97F4A7C15uL.toLong())
        for (b in 2 until 8) data[b] = rng.nextInt(256).toByte()
        return SignalCorrelator.FramePoint(t, data)
    }

    /** Decode a discovered candidate against a frame, the way the app would. */
    private fun decode(c: SignalCorrelator.Candidate, data: ByteArray): Double {
        val sig = DbcSignal(
            "x", c.startBit, c.length, c.byteOrder, c.signed, c.factor, c.offset, 0.0, 0.0, "",
        )
        return SignalDecoder.decode(sig, data)
    }

    // ── Recover a 16-bit speed field and a usable scale ─────────────────────────

    /** speed(t) km/h: 10..90, smooth, 20 s period — variance, non-monotonic. */
    private fun speedKmh(tMs: Long): Double {
        val s = (tMs - t0) / 1000.0
        return 50.0 + 40.0 * sin(2 * PI * s / 20.0)
    }

    @Test
    fun `recovers the speed field and a scale that decodes back to speed`() {
        val frames = (0 until 1200).map { i -> frameFor(i, t0 + i * 50L, speedKmh(t0 + i * 50L)) }
        val ref = (0..60).map { j -> SignalCorrelator.RefPoint(t0 + j * 1000L, speedKmh(t0 + j * 1000L)) }

        val result = SignalCorrelator.correlate(frames, ref, dlc = 8)

        assertEquals(SignalCorrelator.Status.OK, result.status)
        assertTrue("expected candidates", result.candidates.isNotEmpty())

        val top = result.candidates.first()
        assertEquals("startBit", 0, top.startBit)
        assertEquals("length", 16, top.length)
        assertEquals("endianness", ByteOrder.INTEL, top.byteOrder)
        assertTrue("near-perfect correlation (r=${top.correlation})", top.correlation > 0.999)
        assertEquals("recovered factor", factor, top.factor, factor * 0.05)

        // The product that matters: the discovered decoder reproduces true speed.
        for (i in listOf(100, 400, 700, 1000)) {
            val t = t0 + i * 50L
            assertEquals("decode @$i", speedKmh(t), decode(top, frames[i].data), 1.5)
        }
    }

    // ── Lag search recovers a deliberate clock skew ─────────────────────────────

    /** Faster profile so a sub-second skew actually shifts phase. */
    private fun fastSpeedKmh(tMs: Long): Double {
        val s = (tMs - t0) / 1000.0
        return 50.0 + 40.0 * sin(2 * PI * s / 8.0)   // 8 s period
    }

    @Test
    fun `lag search tolerates a 400ms gps clock skew`() {
        val frames = (0 until 1200).map { i -> frameFor(i, t0 + i * 50L, fastSpeedKmh(t0 + i * 50L)) }
        // GPS fix at time t reports the speed the CAN had 400 ms earlier (delayed channel).
        val skewMs = 400L
        val ref = (0..60).map { j ->
            val t = t0 + j * 1000L
            SignalCorrelator.RefPoint(t, fastSpeedKmh(t - skewMs))
        }

        val result = SignalCorrelator.correlate(frames, ref, dlc = 8)
        assertEquals(SignalCorrelator.Status.OK, result.status)

        val top = result.candidates.first()
        assertEquals(0, top.startBit)
        assertEquals(16, top.length)
        // Reaching >0.99 on an 8 s-period wave is only possible if the search shifted to
        // ~the true skew — a 400 ms phase error alone would cap r near 0.95.
        assertTrue("r after lag search (r=${top.correlation})", top.correlation > 0.99)
        assertTrue("recovered lag ~400ms (got ${top.lagMs})", kotlin.math.abs(top.lagMs) in 200..600)
    }

    // ── Specificity: an unrelated field does not produce a false positive ───────

    @Test
    fun `monotonic counter does not correlate with a symmetric speed hump`() {
        // Speed rises 0->100 then falls 100->0, symmetric about the midpoint.
        fun humpKmh(tMs: Long): Double {
            val frac = (tMs - t0).toDouble() / 60_000.0   // 0..1 over 60 s
            return if (frac <= 0.5) 200.0 * frac else 200.0 * (1.0 - frac)
        }
        // Frames carry only a monotonic counter — no speed field at all.
        val frames = (0 until 1200).map { i ->
            val data = ByteArray(8)
            put16(data, 0, i)
            SignalCorrelator.FramePoint(t0 + i * 50L, data)
        }
        val ref = (0..60).map { j -> SignalCorrelator.RefPoint(t0 + j * 1000L, humpKmh(t0 + j * 1000L)) }

        val result = SignalCorrelator.correlate(frames, ref, dlc = 8)
        assertEquals(SignalCorrelator.Status.NO_MATCH, result.status)
    }

    // ── Guard: flat reference is reported, not silently mis-fit ─────────────────

    @Test
    fun `constant speed yields REFERENCE_STATIC`() {
        val frames = (0 until 600).map { i ->
            val data = ByteArray(8)
            put16(data, 0, i and 0xFFFF)
            SignalCorrelator.FramePoint(t0 + i * 50L, data)
        }
        val ref = (0..30).map { j -> SignalCorrelator.RefPoint(t0 + j * 1000L, 42.0) }

        val result = SignalCorrelator.correlate(frames, ref, dlc = 8)
        assertEquals(SignalCorrelator.Status.REFERENCE_STATIC, result.status)
    }
}
