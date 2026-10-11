package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** DriveCurve: 0 inside the deadband, no jump just past it, full stick is full speed, and the sign is kept. */
public class DriveCurveTest {
    private static final double EPS = 1e-9;

    @Test
    public void insideTheDeadbandIsZero() {
        assertEquals(0, DriveCurve.shape(0.05, 0.1, 0.5), EPS);
        assertEquals(0, DriveCurve.shape(-0.1, 0.1, 0.5), EPS);
    }

    @Test
    public void justPastTheDeadbandStartsFromZero() {
        double out = DriveCurve.shape(0.1001, 0.1, 0.5);
        assertTrue("got " + out, out > 0 && out < 0.001);
    }

    @Test
    public void fullStickIsFullSpeedEitherWay() {
        assertEquals(1, DriveCurve.shape(1, 0.1, 0.7), EPS);
        assertEquals(-1, DriveCurve.shape(-1, 0.1, 0.7), EPS);
        // A stick reading just past 1 is still 1.
        assertEquals(1, DriveCurve.shape(1.02, 0.1, 0.7), EPS);
    }

    @Test
    public void theWeightBlendsALineAndACube() {
        // No deadband, so the stick goes straight in.
        assertEquals(0.5, DriveCurve.shape(0.5, 0, 0), EPS);
        assertEquals(0.125, DriveCurve.shape(0.5, 0, 1), EPS);
        assertEquals(0.5 * 0.125 + 0.5 * 0.5, DriveCurve.shape(0.5, 0, 0.5), EPS);
        assertEquals(-0.125, DriveCurve.shape(-0.5, 0, 1), EPS);
    }
}
