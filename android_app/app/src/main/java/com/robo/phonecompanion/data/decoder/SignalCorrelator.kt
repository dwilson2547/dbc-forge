package com.robo.phonecompanion.data.decoder

import com.robo.phonecompanion.data.model.ByteOrder
import com.robo.phonecompanion.data.model.DbcSignal
import kotlin.math.abs

/**
 * Phase 11 brute-force signal discovery. Slides candidate bit-field decodings
 * `(startBit, length, endianness, signed)` across the frames of one CAN ID and
 * correlates each against a ground-truth reference channel (e.g. GPS speed).
 *
 * Both series are resampled onto a uniform time grid; a **lag search** finds the
 * alignment that maximises |r|, which absorbs the cross-clock offset between the
 * phone's GPS and the dongle's CAN timestamps (and any BLE jitter). For matches,
 * linear regression of raw→reference yields the `factor`/`offset` to pre-fill a
 * DBC signal.
 *
 * Reuses [SignalDecoder.extractRaw] for bit extraction.
 */
object SignalCorrelator {

    data class FramePoint(val t: Long, val data: ByteArray)
    data class RefPoint(val t: Long, val v: Double)

    data class Candidate(
        val startBit: Int,
        val length: Int,
        val byteOrder: ByteOrder,
        val signed: Boolean,
        val correlation: Double,   // signed Pearson r at best lag (negative = inverse)
        val lagMs: Long,           // reference lag that maximised |r|
        val factor: Double,        // regression slope: physical = raw*factor + offset
        val offset: Double,
        val sampleCount: Int,
    )

    enum class Status { OK, NO_FRAMES, NO_REFERENCE, REFERENCE_STATIC, NO_OVERLAP, NO_MATCH }

    data class Result(val status: Status, val candidates: List<Candidate>)

    fun correlate(
        frames: List<FramePoint>,
        reference: List<RefPoint>,
        dlc: Int,
        lengths: List<Int> = listOf(8, 12, 16),
        gridMs: Long = 200L,
        maxLagMs: Long = 2000L,
        threshold: Double = 0.8,
        maxResults: Int = 12,
    ): Result {
        if (frames.size < 5 || dlc <= 0) return Result(Status.NO_FRAMES, emptyList())
        val ref = reference.sortedBy { it.t }
        if (ref.size < 5) return Result(Status.NO_REFERENCE, emptyList())
        if (variance(DoubleArray(ref.size) { ref[it].v }, 0, ref.size) < 1e-6)
            return Result(Status.REFERENCE_STATIC, emptyList())

        val sortedFrames = frames.sortedBy { it.t }
        val t0 = maxOf(sortedFrames.first().t, ref.first().t)
        val t1 = minOf(sortedFrames.last().t, ref.last().t)
        if (t1 - t0 < gridMs * 5) return Result(Status.NO_OVERLAP, emptyList())

        // Uniform grid over the overlap window.
        val gridTimes = ArrayList<Long>()
        run { var t = t0; while (t <= t1) { gridTimes.add(t); t += gridMs } }
        val n = gridTimes.size

        // Reference: linear interpolation onto the grid.
        val refGrid = DoubleArray(n) { interp(ref, gridTimes[it]) }

        // CAN: zero-order hold — each slot takes the latest frame at or before its time.
        val frameAtSlot = arrayOfNulls<ByteArray>(n)
        var fi = 0
        var lastData: ByteArray? = null
        for (i in 0 until n) {
            while (fi < sortedFrames.size && sortedFrames[fi].t <= gridTimes[i]) {
                lastData = sortedFrames[fi].data; fi++
            }
            frameAtSlot[i] = lastData
        }
        val firstValid = frameAtSlot.indexOfFirst { it != null }
        if (firstValid < 0 || n - firstValid < 5) return Result(Status.NO_OVERLAP, emptyList())

        val maxLagSlots = (maxLagMs / gridMs).toInt()
        val totalBits = dlc * 8
        val results = ArrayList<Candidate>()
        val raw = DoubleArray(n)

        for (length in lengths) {
            if (length > totalBits) continue
            for (order in ByteOrder.values()) {
                for (startBit in 0 until totalBits) {
                    if (!windowFits(order, startBit, length, dlc)) continue
                    for (signed in booleanArrayOf(false, true)) {
                        val sig = DbcSignal(
                            name = "cand", startBit = startBit, length = length,
                            byteOrder = order, signed = signed, factor = 1.0, offset = 0.0,
                            min = 0.0, max = 0.0, unit = "",
                        )
                        for (i in firstValid until n) {
                            raw[i] = SignalDecoder.extractRaw(sig, frameAtSlot[i]!!).toDouble()
                        }
                        if (variance(raw, firstValid, n) < 1e-9) continue

                        var bestR = 0.0; var bestLag = 0
                        for (lag in -maxLagSlots..maxLagSlots) {
                            val r = pearsonLagged(raw, refGrid, firstValid, n, lag)
                            if (abs(r) > abs(bestR)) { bestR = r; bestLag = lag }
                        }
                        if (abs(bestR) >= threshold) {
                            val reg = regressLagged(raw, refGrid, firstValid, n, bestLag)
                            if (reg.count >= 5) {
                                results.add(
                                    Candidate(
                                        startBit, length, order, signed, bestR,
                                        bestLag.toLong() * gridMs, reg.slope, reg.intercept, reg.count,
                                    )
                                )
                            }
                        }
                    }
                }
            }
        }

        if (results.isEmpty()) return Result(Status.NO_MATCH, emptyList())
        // Rank by correlation, but bucket near-ties (to ~0.001) and prefer the WIDER
        // window within a bucket: a sub-slice that correlates as well as the full field
        // is a less useful (and less accurately scaled) decode than the full field, so
        // it should lose the overlap-suppression contest below.
        val ranked = results.sortedWith(
            compareByDescending<Candidate> { (abs(it.correlation) * 1000).toInt() }
                .thenByDescending { it.length }
                .thenByDescending { abs(it.correlation) }
                .thenBy { it.startBit }
        )
        return Result(Status.OK, suppressOverlaps(ranked, maxResults))
    }

    // ── Geometry / validity ────────────────────────────────────────────────────

    /** Mirror of SignalDecoder.decodeOrNull's bounds check: does the window fit in [dlc] bytes? */
    private fun windowFits(order: ByteOrder, startBit: Int, length: Int, dlc: Int): Boolean {
        val maxByte = when (order) {
            ByteOrder.INTEL -> (startBit + length - 1) / 8
            ByteOrder.MOTOROLA -> {
                var byteIdx = startBit / 8
                var bitIdx = startBit % 8
                var maxByteIdx = byteIdx
                repeat(length) {
                    maxByteIdx = maxOf(maxByteIdx, byteIdx)
                    if (bitIdx == 0) { byteIdx++; bitIdx = 7 } else bitIdx--
                }
                maxByteIdx
            }
        }
        return maxByte < dlc
    }

    /** Absolute bit indices a window occupies — shared by candidates and DBC signals. */
    fun bitsCovered(startBit: Int, length: Int, order: ByteOrder): Set<Int> {
        val bits = HashSet<Int>(length * 2)
        when (order) {
            ByteOrder.INTEL -> for (i in 0 until length) bits.add(startBit + i)
            ByteOrder.MOTOROLA -> {
                var byteIdx = startBit / 8; var bitIdx = startBit % 8
                repeat(length) {
                    bits.add(byteIdx * 8 + bitIdx)
                    if (bitIdx == 0) { byteIdx++; bitIdx = 7 } else bitIdx--
                }
            }
        }
        return bits
    }

    private fun coveredBits(c: Candidate): Set<Int> = bitsCovered(c.startBit, c.length, c.byteOrder)

    /** Keep the highest-|r| candidate among any that share >50% of their bits. */
    private fun suppressOverlaps(ranked: List<Candidate>, maxResults: Int): List<Candidate> {
        val kept = ArrayList<Candidate>()
        val keptBits = ArrayList<Set<Int>>()
        for (c in ranked) {
            val bits = coveredBits(c)
            val clashes = keptBits.any { prev ->
                val inter = bits.count { it in prev }
                inter > minOf(bits.size, prev.size) / 2
            }
            if (!clashes) { kept.add(c); keptBits.add(bits) }
            if (kept.size >= maxResults) break
        }
        return kept
    }

    // ── Stats ───────────────────────────────────────────────────────────────────

    private fun variance(a: DoubleArray, lo: Int, hi: Int): Double {
        val n = hi - lo
        if (n < 2) return 0.0
        var mean = 0.0
        for (i in lo until hi) mean += a[i]
        mean /= n
        var s = 0.0
        for (i in lo until hi) { val d = a[i] - mean; s += d * d }
        return s / n
    }

    /** Pair raw[i] with ref[i+lag] over the valid overlap; signed Pearson r. */
    private fun pearsonLagged(raw: DoubleArray, ref: DoubleArray, lo: Int, n: Int, lag: Int): Double {
        val iStart = maxOf(lo, -lag)
        val iEnd = minOf(n - 1, n - 1 - lag)
        val count = iEnd - iStart + 1
        if (count < 3) return 0.0
        var mx = 0.0; var my = 0.0
        for (i in iStart..iEnd) { mx += raw[i]; my += ref[i + lag] }
        mx /= count; my /= count
        var num = 0.0; var sx = 0.0; var sy = 0.0
        for (i in iStart..iEnd) {
            val dx = raw[i] - mx; val dy = ref[i + lag] - my
            num += dx * dy; sx += dx * dx; sy += dy * dy
        }
        val denom = kotlin.math.sqrt(sx) * kotlin.math.sqrt(sy)
        return if (denom < 1e-9) 0.0 else num / denom
    }

    private data class Regression(val slope: Double, val intercept: Double, val count: Int)

    /** Least-squares fit of reference (y) on raw (x) at [lag]: physical = raw*slope + intercept. */
    private fun regressLagged(raw: DoubleArray, ref: DoubleArray, lo: Int, n: Int, lag: Int): Regression {
        val iStart = maxOf(lo, -lag)
        val iEnd = minOf(n - 1, n - 1 - lag)
        val count = iEnd - iStart + 1
        if (count < 2) return Regression(0.0, 0.0, count)
        var mx = 0.0; var my = 0.0
        for (i in iStart..iEnd) { mx += raw[i]; my += ref[i + lag] }
        mx /= count; my /= count
        var sxx = 0.0; var sxy = 0.0
        for (i in iStart..iEnd) {
            val dx = raw[i] - mx; val dy = ref[i + lag] - my
            sxx += dx * dx; sxy += dx * dy
        }
        val slope = if (sxx < 1e-9) 0.0 else sxy / sxx
        return Regression(slope, my - slope * mx, count)
    }

    /** Linear interpolation of the reference series at time [t]. */
    private fun interp(ref: List<RefPoint>, t: Long): Double {
        if (t <= ref.first().t) return ref.first().v
        if (t >= ref.last().t) return ref.last().v
        // binary search for the bracketing pair
        var lo = 0; var hi = ref.size - 1
        while (hi - lo > 1) {
            val mid = (lo + hi) / 2
            if (ref[mid].t <= t) lo = mid else hi = mid
        }
        val a = ref[lo]; val b = ref[hi]
        if (b.t == a.t) return a.v
        val frac = (t - a.t).toDouble() / (b.t - a.t).toDouble()
        return a.v + frac * (b.v - a.v)
    }
}
