package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

/** Wrapping must work for negative angles too. Java's % keeps the sign (-190 % 360 is -190), so Angles doesn't use it. */
public class AnglesTest {
    private static final double EPS = 1e-9;

    @Test
    public void wrapsIntoMinus180To180() {
        assertEquals(10, Angles.wrapDegrees(370), EPS);
        assertEquals(170, Angles.wrapDegrees(-190), EPS);
        assertEquals(-170, Angles.wrapDegrees(190), EPS);
        assertEquals(5, Angles.wrapDegrees(-715), EPS);
        assertEquals(-90, Angles.wrapDegrees(-90), EPS);
    }

    @Test
    public void aDifferenceTakesTheShortWayAround() {
        // From 170 to -170 is 20 degrees counter-clockwise through 180, not 340 the long way.
        assertEquals(20, Angles.wrapDegrees(-170 - 170), EPS);
        assertEquals(-20, Angles.wrapDegrees(170 - (-170)), EPS);
    }

    @Test
    public void wrapsRadiansToo() {
        assertEquals(-Math.PI / 2, Angles.wrapRadians(1.5 * Math.PI), EPS);
        assertEquals(Math.PI / 2, Angles.wrapRadians(-1.5 * Math.PI), EPS);
    }
}
