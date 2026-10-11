package org.firstinspires.ftc.teamcode.robot;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/**
 * FireControl on its own clock, 10 ms loops: a burst starts once the shot stays ready
 * for READY_HOLD_MS, then feeds without stopping while the wheel sags, pauses only for
 * the aim and BURST_RPM_FLOOR, stops the rail at once on release (the door a little
 * later), and stops after one shot when asked. The shot count sees a ball's quick dip
 * wherever the wheel starts, and not a slow coast or a wheel speeding up.
 */
public class FireControlTest {
    private static final double LOOP_MS = 10;
    private static final double TARGET = 3000;

    private double readyHold, floor, closeDelay, dip, window, noShot;

    private FireControl fire;
    private double now;
    // This loop's inputs. Each test changes them between loops.
    private FireControl.Mode wanted;
    private boolean enoughBalls, stopAfterOneShot, ready, aimed, doorFullyOpen;

    @Before
    public void remember() {
        readyHold = FireControl.READY_HOLD_MS;
        floor = FireControl.BURST_RPM_FLOOR;
        closeDelay = FireControl.DOOR_CLOSE_DELAY_MS;
        dip = FireControl.SHOT_DIP_RPM;
        window = FireControl.DIP_WINDOW_MS;
        noShot = FireControl.NO_SHOT_WARN_MS;

        // The defaults, in case they were tuned.
        FireControl.READY_HOLD_MS = 40;
        FireControl.BURST_RPM_FLOOR = 600;
        FireControl.DOOR_CLOSE_DELAY_MS = 100;
        FireControl.SHOT_DIP_RPM = 150;
        FireControl.DIP_WINDOW_MS = 100;
        FireControl.NO_SHOT_WARN_MS = 400;
        fresh();
    }

    /** A new FireControl, with every input set to "go". */
    private void fresh() {
        fire = new FireControl();
        now = 1000;
        wanted = FireControl.Mode.SHOOT;
        enoughBalls = true;
        stopAfterOneShot = false;
        ready = true;
        aimed = true;
        doorFullyOpen = true;
    }

    @After
    public void putBack() {
        FireControl.READY_HOLD_MS = readyHold;
        FireControl.BURST_RPM_FLOOR = floor;
        FireControl.DOOR_CLOSE_DELAY_MS = closeDelay;
        FireControl.SHOT_DIP_RPM = dip;
        FireControl.DIP_WINDOW_MS = window;
        FireControl.NO_SHOT_WARN_MS = noShot;
    }

    /** One loop, 10 ms after the last, with the wheel at rpm. */
    private void loop(double rpm) {
        now += LOOP_MS;
        fire.update(wanted, enoughBalls, stopAfterOneShot, ready, aimed, rpm, TARGET, doorFullyOpen, now);
    }

    private void loops(int count, double rpm) {
        for (int i = 0; i < count; i++) loop(rpm);
    }

    /** Runs loops with everything ready until the rail feeds. */
    private void startFeeding(double rpm) {
        for (int i = 0; i < 10 && !fire.feeding(); i++) loop(rpm);
        assertTrue("the burst should be feeding by now", fire.feeding());
    }

    @Test
    public void aBurstStartsOnceTheShotStaysReadyForReadyHoldMs() {
        loop(TARGET);
        // 10, 20 and 30 ms of ready: not yet.
        for (int i = 0; i < 3; i++) {
            loop(TARGET);
            assertFalse(fire.doorOpen());
            assertFalse(fire.feeding());
        }
        loop(TARGET);
        assertTrue(fire.doorOpen());
        assertTrue(fire.feeding());

        // A flicker starts the wait again.
        fresh();
        loops(3, TARGET);
        ready = false;
        loop(TARGET);
        ready = true;
        loops(4, TARGET);
        assertFalse(fire.doorOpen());
        loop(TARGET);
        assertTrue(fire.doorOpen());
    }

    @Test
    public void theRailWaitsForTheDoorToOpenFully() {
        doorFullyOpen = false;
        loops(10, TARGET);
        assertTrue(fire.doorOpen());
        assertFalse(fire.feeding());
        doorFullyOpen = true;
        loop(TARGET);
        assertTrue(fire.feeding());
    }

    @Test
    public void feedingGoesOnWhileTheWheelSags400Rpm() {
        startFeeding(TARGET);
        // Out of the ready window now, but ready only starts a burst.
        ready = false;
        for (int i = 1; i <= 20; i++) {
            loop(TARGET - 20 * i);
            assertTrue("loop " + i, fire.feeding());
        }
        loops(20, TARGET - 400);
        assertTrue(fire.feeding());
    }

    @Test
    public void theRailPausesWhileTheAimIsOffAndGoesOnAfter() {
        startFeeding(TARGET);
        aimed = false;
        loop(TARGET);
        assertFalse(fire.feeding());
        assertTrue("the door stays open", fire.doorOpen());
        assertTrue(fire.isFiring());
        assertTrue(fire.status(), fire.status().contains("aim off"));

        aimed = true;
        loop(TARGET);
        assertTrue(fire.feeding());
    }

    @Test
    public void theRailPausesBelowTheRpmFloor() {
        startFeeding(TARGET);
        loop(TARGET - 599);
        assertTrue(fire.feeding());
        loop(TARGET - 601);
        assertFalse(fire.feeding());
        assertTrue(fire.doorOpen());
        loop(TARGET - 300);
        assertTrue(fire.feeding());

        // 0 turns the floor off.
        FireControl.BURST_RPM_FLOOR = 0;
        loop(TARGET - 2000);
        assertTrue(fire.feeding());
    }

    @Test
    public void releaseStopsTheRailAtOnceAndClosesTheDoorAfterTheDelay() {
        startFeeding(TARGET);
        wanted = FireControl.Mode.NONE;
        loop(TARGET);
        assertFalse(fire.feeding());
        assertTrue(fire.doorOpen());
        assertTrue(fire.isFiring());

        loops(9, TARGET);
        assertTrue("90 ms after the rail stopped", fire.doorOpen());
        loop(TARGET);
        assertFalse("100 ms after", fire.doorOpen());
        assertFalse(fire.isFiring());
    }

    @Test
    public void releaseBeforeTheRailRanEndsRightAway() {
        ready = false;
        loops(3, TARGET);
        assertTrue(fire.isFiring());
        wanted = FireControl.Mode.NONE;
        loop(TARGET);
        assertFalse(fire.isFiring());
        assertFalse(fire.doorOpen());
    }

    @Test
    public void switchingToPassMidBurstEndsItAndThePassWaitsForItsOwnShot() {
        startFeeding(TARGET);
        wanted = FireControl.Mode.PASS;
        loop(TARGET);
        assertFalse(fire.feeding());
        loops(10, TARGET);
        assertFalse(fire.isFiring());

        // The next loop starts the pass, which has to be ready for READY_HOLD_MS again.
        loop(TARGET);
        assertTrue(fire.status(), fire.status().startsWith("PASS"));
        assertFalse(fire.doorOpen());
    }

    @Test
    public void stopAfterOneShotEndsTheBurstAfterOneDip() {
        stopAfterOneShot = true;
        startFeeding(TARGET);
        loops(5, TARGET);
        loop(TARGET - 100);
        assertTrue(fire.feeding());
        loop(TARGET - 200);
        assertEquals(1, fire.totalShots());
        assertFalse("the rail stops the loop the shot is counted", fire.feeding());
        assertTrue("the door waits DOOR_CLOSE_DELAY_MS", fire.doorOpen());

        // RobotCommands.shoot() lets go once it sees the shot.
        wanted = FireControl.Mode.NONE;
        loops(10, TARGET - 200);
        assertFalse(fire.isFiring());
        assertEquals(1, fire.totalShots());
    }

    @Test
    public void aSharpDipCountsWhereverTheWheelStarts() {
        // At its target (static hood), and 300 above it (the coax hood's ready window).
        for (double start : new double[]{TARGET, TARGET + 300}) {
            fresh();
            startFeeding(start);
            loops(5, start);
            loop(start - 80);
            assertEquals(0, fire.totalShots());
            loop(start - 160);
            assertEquals("starting at " + start, 1, fire.totalShots());
        }
    }

    @Test
    public void aSlowCoastIsNotAShot() {
        // 300 above target, coasting down 300 RPM over a second.
        startFeeding(TARGET + 300);
        for (int i = 1; i <= 100; i++) loop(TARGET + 300 - 3 * i);
        assertEquals(0, fire.totalShots());
        assertTrue(fire.feeding());
        // The rail ran 400 ms or more with no shot: telemetry says so.
        assertTrue(fire.status(), fire.status().contains("no shot seen"));
    }

    @Test
    public void aWheelSpeedingUpIsNotAShot() {
        // 300 under target, speeding up 20 RPM a loop, with a 43 RPM reading step every other loop.
        startFeeding(TARGET - 300);
        for (int i = 1; i <= 40; i++) loop(TARGET - 300 + 20 * i + (i % 2 == 0 ? 43 : 0));
        assertEquals(0, fire.totalShots());
    }

    @Test
    public void aDeepDipCountsOnceAndTheNextBallCountsAgain() {
        startFeeding(TARGET);
        loops(5, TARGET);
        // 400 down in 50 ms: counted on the way down, not again lower down.
        for (int i = 1; i <= 5; i++) loop(TARGET - 80 * i);
        loops(3, TARGET - 400);
        assertEquals(1, fire.totalShots());

        // Back up to target, then the next ball.
        for (double rpm = TARGET - 360; rpm <= TARGET; rpm += 40) loop(rpm);
        loops(5, TARGET);
        assertEquals(1, fire.totalShots());
        loop(TARGET - 80);
        loop(TARGET - 160);
        assertEquals(2, fire.totalShots());
    }

    @Test
    public void enoughBallsOnlyGatesTheStart() {
        enoughBalls = false;
        loops(10, TARGET);
        assertFalse(fire.isFiring());
        assertFalse(fire.doorOpen());

        enoughBalls = true;
        startFeeding(TARGET);

        // Balls leaving during the burst don't stop it.
        enoughBalls = false;
        loops(5, TARGET);
        assertTrue(fire.feeding());
    }
}
