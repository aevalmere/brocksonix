package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** Our PIDF: each part's sign and size, the integral's limits, and no kicks from target jumps or wrapping. */
public class PIDFTest {
    private static final double EPS = 1e-9;
    // Speeds are a change divided by DT, so they carry a little rounding.
    private static final double RATE_EPS = 1e-6;
    private static final double DT = 0.01;

    @Test
    public void pPushesTowardTheTarget() {
        PIDF pidf = new PIDF(false);
        pidf.setGains(0.5, 0, 0, 0);
        assertEquals(3, pidf.update(10, 4, DT), EPS);
        assertEquals(3, pidf.lastP(), EPS);
        assertEquals(-2, pidf.update(0, 4, DT), EPS);
    }

    @Test
    public void angleErrorTakesTheShortWayAroundAndIncludesTheOffset() {
        PIDF pidf = new PIDF(true);
        pidf.setGains(1, 0, 0, 0);
        // From 170 to -170 is 20 degrees counter-clockwise, through 180.
        assertEquals(20, pidf.update(-170, 170, DT), EPS);
        // A trim (offset) of 5 adds to the target.
        assertEquals(25, pidf.update(-170, 5, 170, 0, DT), EPS);
    }

    @Test
    public void firstUpdateHasNoIOrD() {
        PIDF pidf = new PIDF(true);
        pidf.setGains(1, 1, 1, 0);
        // A long time since the turret was made (init): still only P on the first call.
        assertEquals(10, pidf.update(10, 0, 5.0), EPS);
        assertEquals(0, pidf.lastI(), EPS);
        assertEquals(0, pidf.lastD(), EPS);
    }

    @Test
    public void integralOnlyAddsUpInsideTheZone() {
        PIDF pidf = new PIDF(false);
        pidf.setGains(0, 1, 0, 0);
        pidf.setIntegralLimits(5, 100);
        pidf.update(10, 0, DT);
        pidf.update(10, 0, DT);
        assertEquals("outside the zone", 0, pidf.lastI(), EPS);
        pidf.update(10, 7, DT);
        assertEquals("inside the zone: error 3 for 0.01 s", 0.03, pidf.lastI(), EPS);
        pidf.update(10, 7, DT);
        assertEquals(0.06, pidf.lastI(), EPS);
        pidf.update(10, 0, DT);
        assertEquals("back outside: starts again from 0", 0, pidf.lastI(), EPS);
    }

    @Test
    public void integralIsCapped() {
        PIDF pidf = new PIDF(false);
        pidf.setGains(0, 2, 0, 0);
        pidf.setIntegralLimits(100, 0.5);
        for (int i = 0; i < 50; i++) pidf.update(1, 0, 0.1);
        assertEquals(0.5, pidf.lastI(), EPS);
        // Raising KI live doesn't let what is stored push past the cap either.
        pidf.setGains(0, 4, 0, 0);
        pidf.update(1, 0, 0.1);
        assertEquals(0.5, pidf.lastI(), EPS);
    }

    @Test
    public void integralStartsAgainWhenTheErrorChangesSign() {
        PIDF pidf = new PIDF(false);
        pidf.setGains(0, 1, 0, 0);
        // The first call adds nothing, then 4 x (error 1 for 0.1 s).
        for (int i = 0; i < 5; i++) pidf.update(1, 0, 0.1);
        assertEquals(0.4, pidf.lastI(), EPS);
        // Crossed the target: the old +0.4 is gone, only this loop's -0.1 is left.
        pidf.update(1, 2, 0.1);
        assertEquals(-0.1, pidf.lastI(), EPS);
    }

    @Test
    public void integralDoesNotWindUpWhileTheOutputIsMaxedOut() {
        PIDF pidf = new PIDF(false);
        pidf.setGains(1, 1, 0, 0);
        pidf.setOutputLimit(0.8);
        // P alone (10) is far past the limit, the same way as the error.
        for (int i = 0; i < 20; i++) pidf.update(10, 0, 0.1);
        assertEquals(0, pidf.lastI(), EPS);

        // The extra push counts too: P 0.5 + extra 0.4 is past 0.8.
        PIDF withExtra = new PIDF(false);
        withExtra.setGains(0.05, 1, 0, 0);
        withExtra.setOutputLimit(0.8);
        for (int i = 0; i < 20; i++) withExtra.update(10, 0, 0, 0.4, 0.1);
        assertEquals(0, withExtra.lastI(), EPS);

        // Maxed out the other way (something pushes against the error): adding up helps then, so it does.
        PIDF against = new PIDF(false);
        against.setGains(0.05, 1, 0, 0);
        against.setOutputLimit(0.8);
        against.update(10, 0, 0, -2, 0.1);
        against.update(10, 0, 0, -2, 0.1);
        assertEquals(1.0, against.lastI(), EPS);
    }

    @Test
    public void integralCountsAtMostATenthOfASecondPerLoop() {
        PIDF pidf = new PIDF(false);
        pidf.setGains(0, 1, 0, 0);
        pidf.update(1, 0, DT);
        // A 5 s hiccup adds 1 x 0.1, not 1 x 5.
        pidf.update(1, 0, 5.0);
        assertEquals(0.1, pidf.lastI(), EPS);
    }

    @Test
    public void aPausedIntegralHoldsItsValue() {
        PIDF pidf = new PIDF(false);
        pidf.setGains(0, 1, 0, 0);
        for (int i = 0; i < 3; i++) pidf.update(1, 0, 0.1);
        assertEquals(0.2, pidf.lastI(), EPS);
        pidf.setIntegralPaused(true);
        pidf.update(1, 0, 0.1);
        pidf.update(1, 0, 0.1);
        assertEquals("held, not emptied", 0.2, pidf.lastI(), EPS);
        pidf.setIntegralPaused(false);
        pidf.update(1, 0, 0.1);
        assertEquals(0.3, pidf.lastI(), EPS);
        pidf.resetIntegral();
        pidf.update(1, 0, 0.1);
        assertEquals("emptied, then this loop's 0.1", 0.1, pidf.lastI(), EPS);
    }

    @Test
    public void aTargetJumpDoesNotKickDOrF() {
        PIDF pidf = new PIDF(true);
        pidf.setGains(0, 0, 1, 1);
        pidf.setTargetRateLimits(30, 720);
        pidf.update(0, 0, DT);
        // D-pad 0 to 180.
        pidf.update(180, 0, DT);
        assertEquals(0, pidf.lastD(), EPS);
        assertEquals(0, pidf.lastF(), EPS);
        // Only the turret moving changes D: 1 degree in 0.01 s is 100 deg/s, and D brakes it.
        pidf.update(180, 1, DT);
        assertEquals(-100, pidf.lastD(), RATE_EPS);
    }

    @Test
    public void aTargetJumpStartsTheIntegralAgain() {
        PIDF pidf = new PIDF(true);
        pidf.setGains(0, 1, 0, 0);
        pidf.setTargetRateLimits(30, 720);
        for (int i = 0; i < 5; i++) pidf.update(2, 0, 0.1);
        assertEquals(0.8, pidf.lastI(), EPS);
        // New target 48 degrees away (the error stays 2): only this loop's 2 x 0.1 is left.
        pidf.update(50, 48, 0.1);
        assertEquals(0.2, pidf.lastI(), EPS);
    }

    @Test
    public void aTrimChangeDoesNotKickDOrF() {
        PIDF pidf = new PIDF(true);
        pidf.setGains(1, 0, 1, 1);
        pidf.update(10, 0, 0, 0, DT);
        pidf.update(10, 2, 0, 0, DT);
        assertEquals("the trim still moves the error", 12, pidf.lastP(), EPS);
        assertEquals(0, pidf.lastD(), EPS);
        assertEquals(0, pidf.lastF(), EPS);
    }

    @Test
    public void speedsDoNotSpikeWhenAnglesWrapAround() {
        PIDF pidf = new PIDF(true);
        pidf.setGains(0, 0, 1, 0);
        pidf.update(180, 179, DT);
        // The turret goes 179 to -179: 2 degrees through 180, not 358 back.
        pidf.update(180, -179, DT);
        assertEquals(-200, pidf.lastD(), RATE_EPS);

        PIDF target = new PIDF(true);
        target.setGains(0, 0, 0, 1);
        target.setTargetRateLimits(30, 720);
        target.update(179, 0, DT);
        // The target goes 179 to -179: 2 degrees, a move, not a jump.
        target.update(-179, 0, DT);
        assertEquals(200, target.lastF(), RATE_EPS);
    }

    @Test
    public void targetSpeedIsCappedAndAJumpCountsAsZero() {
        PIDF pidf = new PIDF(true);
        pidf.setGains(0, 0, 0, 1);
        pidf.setTargetRateLimits(30, 720);
        pidf.update(0, 0, DT);
        // 20 degrees in 0.01 s is 2000 deg/s: capped.
        pidf.update(20, 0, DT);
        assertEquals(720, pidf.lastF(), RATE_EPS);
        // 40 degrees in one loop: a jump.
        pidf.update(60, 0, DT);
        assertEquals(0, pidf.lastF(), EPS);
        // Moving again at 500 deg/s.
        pidf.update(65, 0, DT);
        assertEquals(500, pidf.lastF(), RATE_EPS);
    }

    @Test
    public void filtersSmoothTheSpeeds() {
        PIDF target = new PIDF(false);
        target.setGains(0, 0, 0, 1);
        target.setFilters(0, 0.5);
        target.update(0, 0, DT);
        // The target moves at 100 per second: the smoothed speed goes half way each loop.
        target.update(1, 0, DT);
        assertEquals(50, target.lastF(), RATE_EPS);
        target.update(2, 0, DT);
        assertEquals(75, target.lastF(), RATE_EPS);

        PIDF measured = new PIDF(false);
        measured.setGains(0, 0, 1, 0);
        measured.setFilters(0.5, 0);
        measured.update(0, 0, DT);
        measured.update(0, 1, DT);
        assertEquals(-50, measured.lastD(), RATE_EPS);
        measured.update(0, 2, DT);
        assertEquals(-75, measured.lastD(), RATE_EPS);
    }

    @Test
    public void aBadReadingKeepsTheLastSpeed() {
        PIDF pidf = new PIDF(true);
        pidf.setGains(0, 0, 1, 0);
        pidf.setMaxMeasuredRate(2000);
        pidf.update(0, 0, DT);
        pidf.update(0, 1, DT);
        assertEquals(-100, pidf.lastD(), RATE_EPS);
        // 60 degrees in 0.01 s (6000 deg/s) can't be real.
        pidf.update(0, 61, DT);
        assertEquals(-100, pidf.lastD(), RATE_EPS);
        // The jump back from the bad reading is skipped too.
        pidf.update(0, 2, DT);
        assertEquals(-100, pidf.lastD(), RATE_EPS);
    }

    @Test
    public void aNaNTargetDoesNotStick() {
        PIDF pidf = new PIDF(true);
        pidf.setGains(1, 1, 1, 1);
        pidf.update(10, 0, DT);
        assertTrue(Double.isNaN(pidf.update(Double.NaN, 0, DT)));
        // The next good target gives a number again at once.
        assertFalse(Double.isNaN(pidf.update(10, 0, DT)));
        assertFalse(Double.isNaN(pidf.lastI()));
        assertFalse(Double.isNaN(pidf.update(10, 0, DT)));
    }
}
