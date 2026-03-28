package com.example.navsight1

import org.junit.Assert.assertEquals
import org.junit.Before
import org.junit.Test
import java.lang.reflect.Method

/**
 * Unit tests for OpticalFlowProcessor.determineDirection
 *
 * Camera is mounted face-down on vehicle with x-axis aligned to forward direction:
 *   dx positive (floor moves right in image) = FORWARD
 *   dx negative (floor moves left in image)  = BACKWARD
 *   dy positive (floor moves down in image)  = RIGHT
 *   dy negative (floor moves up in image)    = LEFT
 *
 * Story 1.1 AC1/AC2: direction badge must show correct Hebrew label matching actual movement.
 */
class OpticalFlowProcessorTest {

    private lateinit var processor: OpticalFlowProcessor
    private lateinit var determineDirection: Method

    private val MIN_MOVEMENT = 0.8f  // MIN_MOVEMENT_WALK constant

    @Before
    fun setUp() {
        processor = OpticalFlowProcessor()
        determineDirection = OpticalFlowProcessor::class.java
            .getDeclaredMethod("determineDirection", Float::class.java, Float::class.java, Float::class.java, Float::class.java)
            .also { it.isAccessible = true }
    }

    private fun direction(dx: Float, dy: Float, magnitude: Float, minMovement: Float = MIN_MOVEMENT): OpticalFlowProcessor.MovementDirection {
        return determineDirection.invoke(processor, dx, dy, magnitude, minMovement)
                as OpticalFlowProcessor.MovementDirection
    }

    // ── STOPPED ──────────────────────────────────────────────────────────────

    @Test
    fun stoppedWhenMagnitudeBelowThreshold() {
        assertEquals(OpticalFlowProcessor.MovementDirection.STOPPED, direction(5f, 0f, 0.5f))
    }

    @Test
    fun stoppedWhenMagnitudeBelowThreshold2() {
        // Clearly below threshold
        assertEquals(OpticalFlowProcessor.MovementDirection.STOPPED, direction(5f, 0f, MIN_MOVEMENT - 0.01f))
    }

    @Test
    fun notStoppedWhenMagnitudeEqualsThreshold() {
        // Boundary: exactly at MIN_MOVEMENT is NOT stopped (guard is strict <)
        val result = direction(5f, 0f, MIN_MOVEMENT)
        assert(result != OpticalFlowProcessor.MovementDirection.STOPPED) {
            "magnitude == MIN_MOVEMENT should not return STOPPED (guard is strict <)"
        }
    }

    // ── FORWARD ──────────────────────────────────────────────────────────────

    @Test
    fun forwardWhenDxPositiveDominant() {
        // Pure forward motion: large positive dx, zero dy
        assertEquals(OpticalFlowProcessor.MovementDirection.FORWARD, direction(5f, 0f, 5f))
    }

    @Test
    fun forwardWhenDxPositiveDominantWithSmallDy() {
        // dx clearly dominant: absDx > absDy * 0.7
        assertEquals(OpticalFlowProcessor.MovementDirection.FORWARD, direction(3f, 1f, 3.2f))
    }

    // ── BACKWARD ─────────────────────────────────────────────────────────────

    @Test
    fun backwardWhenDxNegativeDominant() {
        assertEquals(OpticalFlowProcessor.MovementDirection.BACKWARD, direction(-5f, 0f, 5f))
    }

    @Test
    fun backwardWhenDxNegativeDominantWithSmallDy() {
        assertEquals(OpticalFlowProcessor.MovementDirection.BACKWARD, direction(-3f, 1f, 3.2f))
    }

    // ── RIGHT ─────────────────────────────────────────────────────────────────

    @Test
    fun rightWhenDyPositiveDominant() {
        // dy positive = floor moves down = RIGHT
        assertEquals(OpticalFlowProcessor.MovementDirection.RIGHT, direction(0f, 5f, 5f))
    }

    @Test
    fun rightWhenDyPositiveDominantWithSmallDx() {
        assertEquals(OpticalFlowProcessor.MovementDirection.RIGHT, direction(1f, 3f, 3.2f))
    }

    // ── LEFT ──────────────────────────────────────────────────────────────────

    @Test
    fun leftWhenDyNegativeDominant() {
        // dy negative = floor moves up = LEFT
        assertEquals(OpticalFlowProcessor.MovementDirection.LEFT, direction(0f, -5f, 5f))
    }

    @Test
    fun leftWhenDyNegativeDominantWithSmallDx() {
        assertEquals(OpticalFlowProcessor.MovementDirection.LEFT, direction(1f, -3f, 3.2f))
    }

    // ── BRANCH 1 dominant (absDx > absDy * 0.7) ─────────────────────────────

    @Test
    fun forwardWhenDxDominantOverDy() {
        // absDx=3 > absDy*0.7=2.1 → branch 1 → FORWARD
        assertEquals(OpticalFlowProcessor.MovementDirection.FORWARD, direction(3f, 3f, 4.2f))
    }

    @Test
    fun backwardWhenNegativeDxDominantOverDy() {
        // absDx=3 > absDy*0.7=2.1 → branch 1 → BACKWARD
        assertEquals(OpticalFlowProcessor.MovementDirection.BACKWARD, direction(-3f, 3f, 4.2f))
    }

    // ── BRANCH 2 dominant (absDy > absDx * 0.7) ─────────────────────────────

    @Test
    fun rightWhenDyDominantOverDx() {
        // absDy=3 > absDx*0.7=1.4, absDx=2 not > absDy*0.7=2.1 → branch 2 → RIGHT
        assertEquals(OpticalFlowProcessor.MovementDirection.RIGHT, direction(2f, 3f, 3.6f))
    }

    @Test
    fun leftWhenNegativeDyDominantOverDx() {
        // absDy=3 > absDx*0.7=1.4 → branch 2 → LEFT
        assertEquals(OpticalFlowProcessor.MovementDirection.LEFT, direction(2f, -3f, 3.6f))
    }

    // ── ZERO inputs ───────────────────────────────────────────────────────────

    @Test
    fun stoppedWhenBothZeroWithNonZeroMagnitude() {
        // dx=0, dy=0 with magnitude above threshold must return STOPPED, not BACKWARD
        assertEquals(OpticalFlowProcessor.MovementDirection.STOPPED, direction(0f, 0f, 5f))
    }

    // ── NaN guard ─────────────────────────────────────────────────────────────

    @Test
    fun stoppedWhenMagnitudeIsNaN() {
        assertEquals(OpticalFlowProcessor.MovementDirection.STOPPED, direction(1f, 0f, Float.NaN))
    }

    @Test
    fun stoppedWhenDxIsNaN() {
        assertEquals(OpticalFlowProcessor.MovementDirection.STOPPED, direction(Float.NaN, 0f, 5f))
    }

    // ── REGRESSION: previous broken mapping must NOT reappear ─────────────────

    @Test
    fun regressionForwardMustNotBecomeLeft() {
        // Before fix: large positive dx returned LEFT. Must not regress.
        val result = direction(10f, 0f, 10f)
        assert(result != OpticalFlowProcessor.MovementDirection.LEFT) {
            "Regression: dx=10, dy=0 returned LEFT — axis-swap fix was lost"
        }
        assertEquals(OpticalFlowProcessor.MovementDirection.FORWARD, result)
    }

    @Test
    fun regressionBackwardMustNotBecomeRight() {
        val result = direction(-10f, 0f, 10f)
        assert(result != OpticalFlowProcessor.MovementDirection.RIGHT) {
            "Regression: dx=-10, dy=0 returned RIGHT — axis-swap fix was lost"
        }
        assertEquals(OpticalFlowProcessor.MovementDirection.BACKWARD, result)
    }
}
