package org.firstinspires.ftc.teamcode.robot.shot;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.util.LookupTable;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/**
 * HoodSolver: every angle it gives scores and clears the lip (also when the exit
 * speed changes with the angle), it follows the measured RPM without jumping,
 * bad inputs give NaN, flight time, seeds and passes agree with the physics, and
 * ShotTables checks the coax tables' order.
 */
public class HoodSolverTest {
    // CoaxHood.MIN_LAUNCH_DEG and MAX_LAUNCH_DEG's defaults.
    private static final double MIN_DEG = 40;
    private static final double MAX_DEG = 75;
    private static final double MAX_RPM = 5500;
    private static final double EPS = 1e-9;

    private double[][] coaxRpm, exitSpeed, exitSpeedAngle;
    private double targetHeight, launchHeight, seedWindow, targetBand, rpmMargin;

    @Before
    public void remember() {
        coaxRpm = ShotTables.COAX_RPM;
        exitSpeed = ShotTables.COAX_EXIT_SPEED;
        exitSpeedAngle = ShotTables.COAX_EXIT_SPEED_ANGLE;
        targetHeight = HoodSolver.TARGET_HEIGHT;
        launchHeight = HoodSolver.LAUNCH_HEIGHT;
        seedWindow = HoodSolver.SEED_RPM_WINDOW;
        targetBand = HoodSolver.TARGET_BAND;
        rpmMargin = HoodSolver.RPM_MARGIN;
    }

    @After
    public void putBack() {
        ShotTables.COAX_RPM = coaxRpm;
        ShotTables.COAX_EXIT_SPEED = exitSpeed;
        ShotTables.COAX_EXIT_SPEED_ANGLE = exitSpeedAngle;
        HoodSolver.TARGET_HEIGHT = targetHeight;
        HoodSolver.LAUNCH_HEIGHT = launchHeight;
        HoodSolver.SEED_RPM_WINDOW = seedWindow;
        HoodSolver.TARGET_BAND = targetBand;
        HoodSolver.RPM_MARGIN = rpmMargin;
    }

    @Test
    public void everyAngleIsInRangeCrossesInsideTheBandAndClearsTheLip() {
        checkEveryAngleScores();
    }

    @Test
    public void everyAngleScoresWhenTheExitSpeedChangesWithTheAngle() {
        // 15% slower at the flattest angle, 15% faster at the steepest.
        ShotTables.COAX_EXIT_SPEED_ANGLE = new double[][]{{40, 0.85}, {75, 1.15}};
        checkEveryAngleScores();
    }

    /**
     * Over distances 20 to 110 and each distance's COAX_RPM ± 300: any angle given
     * is in range, crosses in the band and clears the lip, also with the wheel
     * RPM_MARGIN slower or faster.
     */
    private static void checkEveryAngleScores() {
        int tried = 0, found = 0;
        for (double distance = 20; distance <= 110; distance += 5) {
            double tableRpm = LookupTable.at(ShotTables.COAX_RPM, distance);
            for (double offset = -300; offset <= 300; offset += 50) {
                double rpm = tableRpm + offset;
                double angle = HoodSolver.launchDeg(distance, rpm, MIN_DEG, MAX_DEG);
                tried++;
                if (Double.isNaN(angle)) continue;
                found++;
                String where = distance + " in. at " + rpm + " RPM: " + angle + "°";
                assertTrue(where, angle >= MIN_DEG && angle <= MAX_DEG);

                // With the speed at this very angle, at the RPM and RPM_MARGIN either side.
                assertTrue(where, scoresAt(distance, rpm, angle));
                assertTrue(where + ", wheel slower", scoresAt(distance, rpm - HoodSolver.RPM_MARGIN, angle));
                assertTrue(where + ", wheel faster", scoresAt(distance, rpm + HoodSolver.RPM_MARGIN, angle));
            }
        }
        // So the test can't pass by finding nothing.
        assertTrue("found " + found + " of " + tried, found > tried / 2);
    }

    /** The test's own check: with the speed at this angle, the ball crosses inside the band and passes above the lip. */
    private static boolean scoresAt(double distance, double rpm, double angle) {
        double speed = HoodSolver.exitSpeed(rpm, angle);
        double height = HoodSolver.heightAt(distance, angle, speed);
        if (!(Math.abs(height - HoodSolver.TARGET_HEIGHT) <= Math.abs(HoodSolver.TARGET_BAND) + 1e-6)) return false;
        double lipDistance = distance - HoodSolver.LIP_BEFORE;
        return lipDistance <= 0 || HoodSolver.heightAt(lipDistance, angle, speed) >= HoodSolver.LIP_HEIGHT;
    }

    private static boolean scoresWithMarginAt(double distance, double rpm, double angle) {
        double margin = HoodSolver.RPM_MARGIN;
        return scoresAt(distance, rpm, angle) && scoresAt(distance, rpm - margin, angle) && scoresAt(distance, rpm + margin, angle);
    }

    @Test
    public void marginRefusesAnAngleThatOnlyScoresAtTheExactRpm() {
        // The slowest RPM that scores from 80 in. with no margin: the angle there changes fast with the RPM.
        HoodSolver.RPM_MARGIN = 0;
        double slowest = Double.NaN;
        for (double rpm = 500; rpm <= MAX_RPM && Double.isNaN(slowest); rpm += 25) {
            if (!Double.isNaN(HoodSolver.launchDeg(80, rpm, MIN_DEG, MAX_DEG))) slowest = rpm;
        }
        assertFalse(Double.isNaN(slowest));
        // 50 RPM slower, nothing reaches the CELL, so with the margin that RPM doesn't fire.
        HoodSolver.RPM_MARGIN = 50;
        assertTrue(Double.isNaN(HoodSolver.launchDeg(80, slowest, MIN_DEG, MAX_DEG)));
        assertFalse(Double.isNaN(HoodSolver.launchDeg(80, slowest + 100, MIN_DEG, MAX_DEG)));
    }

    /**
     * Up close the sampled heights can all fall between the band and the lip. Any
     * RPM where a plain walk through the hood's range finds an angle that scores
     * with the margin must get an angle from launchDeg too. Once with the margin,
     * once without (where more RPMs score).
     */
    @Test
    public void closeUpFindsEveryAngleAWalkFinds() {
        double[] margins = {rpmMargin, 0};
        for (double margin : margins) {
            HoodSolver.RPM_MARGIN = margin;
            int walkFinds = 0;
            for (double rpm = 1750; rpm <= 2500; rpm += 25) {
                boolean found = false;
                for (int step = 0; MIN_DEG + 0.1 * step <= MAX_DEG + EPS && !found; step++) {
                    found = scoresWithMarginAt(20, rpm, Math.min(MIN_DEG + 0.1 * step, MAX_DEG));
                }
                double angle = HoodSolver.launchDeg(20, rpm, MIN_DEG, MAX_DEG);
                String where = "margin " + margin + ", 20 in. at " + rpm + " RPM: " + angle + "°";
                if (found) {
                    walkFinds++;
                    assertFalse(where, Double.isNaN(angle));
                }
                if (!Double.isNaN(angle)) assertTrue(where, scoresWithMarginAt(20, rpm, angle));
            }
            assertTrue("the walk finds something with margin " + margin, walkFinds > 0);
        }
    }

    @Test
    public void bandOfZeroStillScoresAtTheCenter() {
        // A band of 0 leaves no room for the margin either, so both go to 0.
        HoodSolver.TARGET_BAND = 0;
        HoodSolver.RPM_MARGIN = 0;
        for (double distance = 35; distance <= 95; distance += 15) {
            double rpm = LookupTable.at(ShotTables.COAX_RPM, distance);
            double angle = HoodSolver.launchDeg(distance, rpm, MIN_DEG, MAX_DEG);
            String where = distance + " in. at " + rpm + " RPM";
            assertFalse(where, Double.isNaN(angle));
            double height = HoodSolver.heightAt(distance, angle, HoodSolver.exitSpeed(rpm, angle));
            assertEquals(where, HoodSolver.TARGET_HEIGHT, height, 1e-6);
        }
    }

    @Test
    public void settlesOnTheAngleForItsOwnSpeed() {
        // At the 80 in. seed the center's low arc is in range, so the answer should cross right at the center,
        // even with a speed that changes 5% either way with the angle.
        ShotTables.COAX_EXIT_SPEED_ANGLE = new double[][]{{40, 0.95}, {75, 1.05}};
        double rpm = LookupTable.at(ShotTables.COAX_RPM, 80);
        double angle = HoodSolver.launchDeg(80, rpm, MIN_DEG, MAX_DEG);
        assertFalse(Double.isNaN(angle));
        double height = HoodSolver.heightAt(80, angle, HoodSolver.exitSpeed(rpm, angle));
        assertEquals(HoodSolver.TARGET_HEIGHT, height, 0.1);
    }

    @Test
    public void noAngleForBadInputs() {
        assertTrue(Double.isNaN(HoodSolver.launchDeg(80, 0, MIN_DEG, MAX_DEG)));
        assertTrue(Double.isNaN(HoodSolver.launchDeg(0, 2800, MIN_DEG, MAX_DEG)));
        assertTrue(Double.isNaN(HoodSolver.launchDeg(-10, 2800, MIN_DEG, MAX_DEG)));
        assertTrue(Double.isNaN(HoodSolver.launchDeg(Double.NaN, 2800, MIN_DEG, MAX_DEG)));
        assertTrue(Double.isNaN(HoodSolver.launchDeg(80, Double.NaN, MIN_DEG, MAX_DEG)));
        assertTrue(Double.isNaN(HoodSolver.launchDeg(80, 2800, Double.NaN, MAX_DEG)));
        assertTrue(Double.isNaN(HoodSolver.launchDeg(80, 2800, MIN_DEG, Double.NaN)));
        assertTrue(Double.isNaN(HoodSolver.launchDeg(Double.POSITIVE_INFINITY, 2800, MIN_DEG, MAX_DEG)));
        assertTrue(Double.isNaN(HoodSolver.launchDeg(80, Double.POSITIVE_INFINITY, MIN_DEG, MAX_DEG)));
        // The hood's range upside down.
        assertTrue(Double.isNaN(HoodSolver.launchDeg(80, 2800, MAX_DEG, MIN_DEG)));
        // A NaN typed into Panels.
        HoodSolver.TARGET_HEIGHT = Double.NaN;
        assertTrue(Double.isNaN(HoodSolver.launchDeg(80, 2800, MIN_DEG, MAX_DEG)));
        HoodSolver.TARGET_HEIGHT = targetHeight;
        HoodSolver.LAUNCH_HEIGHT = Double.POSITIVE_INFINITY;
        assertTrue(Double.isNaN(HoodSolver.launchDeg(80, 2800, MIN_DEG, MAX_DEG)));
        HoodSolver.LAUNCH_HEIGHT = launchHeight;
        HoodSolver.RPM_MARGIN = Double.NaN;
        assertTrue(Double.isNaN(HoodSolver.launchDeg(80, 2800, MIN_DEG, MAX_DEG)));
    }

    @Test
    public void followsTheMeasuredRpm() {
        double distance = 80;
        double tableRpm = LookupTable.at(ShotTables.COAX_RPM, distance);
        double atTable = HoodSolver.launchDeg(distance, tableRpm, MIN_DEG, MAX_DEG);
        double dipped = HoodSolver.launchDeg(distance, tableRpm - 200, MIN_DEG, MAX_DEG);
        assertFalse(Double.isNaN(atTable));
        assertFalse("a wheel 200 RPM slow still scores", Double.isNaN(dipped));
        // A slower ball needs a steeper launch to get as high.
        assertTrue(atTable + " then " + dipped, dipped > atTable + 0.5);
        // The same RPM always gives the same answer, either way the sign is.
        assertEquals(dipped, HoodSolver.launchDeg(distance, -(tableRpm - 200), MIN_DEG, MAX_DEG), 0);
    }

    /**
     * As the RPM changes 25 at a time, the angle never jumps more than 5°. The
     * biggest real steps (about 4°) come at the slowest RPM that still reaches the
     * band: there the low and high arcs meet, so the angle swings fastest with
     * speed. Flipping onto a lob would be a jump of 30° or more.
     *
     * This covers COAX_RPM's distances, 20 to 110 in. Past about 111 in. the low
     * arc runs out at the flattest hood angle once the wheel is fast enough
     * (about 2600 RPM), and the next scoring angle there is a lob.
     */
    @Test
    public void angleNeverJumpsAsTheRpmChanges() {
        for (double distance = 20; distance <= 110; distance += 5) {
            checkNoJumps(distance, MIN_DEG, MAX_DEG);
        }
    }

    @Test
    public void lowArcsComeBeforeAnyHighArc() {
        // With a hood that reaches 20° to 88°, trying the high arc at the center before
        // the low arcs off center (Ditto's order) flips 22° to 27° here as the RPM rises.
        checkNoJumps(25, 20, 88);
        checkNoJumps(30, 20, 88);
    }

    @Test
    public void tooFastForTheHoodNeverFlipsOntoALob() {
        // Past the end of the table, and with a hood that reaches 80°, a wheel too fast for
        // the flattest angle used to find a lob 30° or more away. Now it waits instead.
        for (double distance = 90; distance <= 130; distance += 5) {
            checkNoJumps(distance, MIN_DEG, MAX_DEG);
            checkNoJumps(distance, MIN_DEG, 80);
        }
    }

    /** Every 25 RPM from 500 to MAX_RPM, each angle found is within 5° of the one before it. */
    private static void checkNoJumps(double distance, double minDeg, double maxDeg) {
        double previous = Double.NaN;
        int found = 0;
        for (double rpm = 500; rpm <= MAX_RPM; rpm += 25) {
            double angle = HoodSolver.launchDeg(distance, rpm, minDeg, maxDeg);
            if (Double.isNaN(angle)) continue;
            found++;
            if (!Double.isNaN(previous)) {
                assertTrue(distance + " in. at " + rpm + " RPM: " + previous + "° then " + angle + "°",
                        Math.abs(angle - previous) <= 5);
            }
            previous = angle;
        }
        assertTrue("something scores from " + distance + " in.", found > 0);
    }

    @Test
    public void flightTimeIsDistanceOverForwardSpeed() {
        double distance = 80, rpm = 2225;
        double angle = HoodSolver.launchDeg(distance, rpm, MIN_DEG, MAX_DEG);
        double speed = HoodSolver.exitSpeed(rpm, angle);
        double seconds = HoodSolver.flightTime(distance, rpm, angle);
        double radians = Math.toRadians(angle);
        assertEquals(distance / (speed * Math.cos(radians)), seconds, 1e-12);

        // Thrown and timed: after that many seconds the ball is where heightAt says.
        double y = HoodSolver.LAUNCH_HEIGHT + speed * Math.sin(radians) * seconds
                - HoodSolver.GRAVITY * seconds * seconds / 2;
        assertEquals(HoodSolver.heightAt(distance, angle, speed), y, 1e-9);

        // A ball 20% slower at every angle takes 1 / 0.8 as long.
        ShotTables.COAX_EXIT_SPEED_ANGLE = new double[][]{{40, 0.8}, {75, 0.8}};
        assertEquals(seconds / 0.8, HoodSolver.flightTime(distance, rpm, angle), 1e-9);

        assertTrue(Double.isNaN(HoodSolver.flightTime(distance, 0, angle)));
        assertTrue(Double.isNaN(HoodSolver.flightTime(distance, rpm, 90)));
        assertTrue(Double.isNaN(HoodSolver.flightTime(distance, rpm, Double.NaN)));
        assertTrue(Double.isNaN(HoodSolver.flightTime(Double.NaN, rpm, angle)));
        assertTrue(Double.isNaN(HoodSolver.flightTime(distance, Double.NaN, angle)));
    }

    @Test
    public void heightAtNeverGetsThereStraightUpOrWithNoSpeed() {
        assertEquals(Double.NEGATIVE_INFINITY, HoodSolver.heightAt(50, 90, 300), 0);
        assertEquals(Double.NEGATIVE_INFINITY, HoodSolver.heightAt(50, 45, 0), 0);
        assertEquals(Double.NEGATIVE_INFINITY, HoodSolver.heightAt(50, 45, Double.NaN), 0);
        // At 0 in. it's the launch height.
        assertEquals(HoodSolver.LAUNCH_HEIGHT, HoodSolver.heightAt(0, 30, 300), EPS);
    }

    @Test
    public void seedIsTheSlowestRpmWithTheWholeWindow() {
        double window = HoodSolver.SEED_RPM_WINDOW;
        for (double distance = 20; distance <= 110; distance += 5) {
            double rpm = HoodSolver.seedRpm(distance, MIN_DEG, MAX_DEG, MAX_RPM);
            double angle = HoodSolver.seedLaunchDeg(distance, MIN_DEG, MAX_DEG, MAX_RPM);
            String where = distance + " in.: " + rpm + " RPM at " + angle + "°";
            assertFalse(where, Double.isNaN(rpm));
            assertTrue(where, rpm <= MAX_RPM);
            assertEquals(where, HoodSolver.launchDeg(distance, rpm, MIN_DEG, MAX_DEG), angle, 0);
            assertFalse(where, Double.isNaN(angle));

            double[] run = longestScoringRun(distance, MAX_RPM);
            if (run[1] - run[0] >= 2 * window) {
                assertTrue(where, scoresAcross(distance, rpm - window, rpm + window));
                // The slowest: 25 RPM lower, the bottom of the window doesn't score.
                double below = rpm - window - 25;
                assertTrue(where, below < 500 || Double.isNaN(HoodSolver.launchDeg(distance, below, MIN_DEG, MAX_DEG)));
            } else {
                assertEquals(where, middle(run), rpm, 0);
            }
        }
    }

    @Test
    public void seedFallsBackToTheMiddleOfTheLongestRun() {
        // No RPM has 5000 RPM of room either way.
        HoodSolver.SEED_RPM_WINDOW = 5000;
        double[] run = longestScoringRun(80, MAX_RPM);
        assertEquals(middle(run), HoodSolver.seedRpm(80, MIN_DEG, MAX_DEG, MAX_RPM), 0);
    }

    @Test
    public void seedStaysUnderTheCapAndIsNaNWhenNothingScores() {
        double rpm = HoodSolver.seedRpm(80, MIN_DEG, MAX_DEG, 2200);
        assertTrue(rpm + " RPM", rpm <= 2200);
        assertFalse(Double.isNaN(HoodSolver.launchDeg(80, rpm, MIN_DEG, MAX_DEG)));

        // Nothing up to 1000 RPM reaches the CELL from 80 in.
        assertTrue(Double.isNaN(HoodSolver.seedRpm(80, MIN_DEG, MAX_DEG, 1000)));
        assertTrue(Double.isNaN(HoodSolver.seedLaunchDeg(80, MIN_DEG, MAX_DEG, 1000)));
        assertTrue(Double.isNaN(HoodSolver.seedRpm(0, MIN_DEG, MAX_DEG, MAX_RPM)));
        assertTrue(Double.isNaN(HoodSolver.seedRpm(80, MIN_DEG, MAX_DEG, 0)));
        assertTrue(Double.isNaN(HoodSolver.seedRpm(80, MAX_DEG, MIN_DEG, MAX_RPM)));
        assertTrue(Double.isNaN(HoodSolver.seedRpm(Double.NaN, MIN_DEG, MAX_DEG, MAX_RPM)));
        assertTrue(Double.isNaN(HoodSolver.seedLaunchDeg(80, MIN_DEG, MAX_DEG, Double.NaN)));
    }

    /**
     * Each COAX_RPM row leaves the hood room for the wheel to be SEED_RPM_WINDOW
     * off either way. Where no RPM has that much room (20, 95 and 110 in. with the
     * defaults), the row is the middle of the longest run of RPMs that score.
     *
     * This only guards the physics seeds while COAX_RPM is still seeded. It is
     * expected to fail once the rows are tuned from real shots, or LAUNCH_HEIGHT,
     * TARGET_BAND or COAX_EXIT_SPEED are measured (step 16). Then re-seed from Coax
     * Hood Test's init screen, or, if the tuned rows are right, delete this test.
     */
    @Test
    public void everyCoaxRpmRowHasRoomBothWays() {
        double window = HoodSolver.SEED_RPM_WINDOW;
        for (double[] row : ShotTables.COAX_RPM) {
            double distance = row[0], rpm = row[1];
            double[] run = longestScoringRun(distance, MAX_RPM);
            String where = "COAX_RPM no longer matches the physics seeds (fine after step 16: re-seed, or delete this test). "
                    + distance + " in. at " + rpm + " RPM, longest run " + run[0] + " to " + run[1];
            if (run[1] - run[0] >= 2 * window) {
                assertTrue(where, scoresAcross(distance, rpm - window, rpm + window));
            } else {
                assertEquals(where, middle(run), rpm, 25);
            }
        }
    }

    /** The lowest and highest RPM of the longest unbroken run that scores, every 25 RPM from 500 to maxRpm. */
    private static double[] longestScoringRun(double distance, double maxRpm) {
        double bestStart = Double.NaN, bestEnd = Double.NaN, start = Double.NaN;
        for (double rpm = 500; rpm <= maxRpm; rpm += 25) {
            boolean scores = !Double.isNaN(HoodSolver.launchDeg(distance, rpm, MIN_DEG, MAX_DEG));
            if (!scores) {
                start = Double.NaN;
                continue;
            }
            if (Double.isNaN(start)) start = rpm;
            if (Double.isNaN(bestStart) || rpm - start > bestEnd - bestStart) {
                bestStart = start;
                bestEnd = rpm;
            }
        }
        return new double[]{bestStart, bestEnd};
    }

    /** The run's middle, on the 25 RPM steps (the lower one when there are two). */
    private static double middle(double[] run) {
        int steps = (int) Math.round((run[1] - run[0]) / 25);
        return run[0] + 25 * (steps / 2);
    }

    private static boolean scoresAcross(double distance, double fromRpm, double toRpm) {
        for (double rpm = fromRpm; rpm <= toRpm + EPS; rpm += 25) {
            if (Double.isNaN(HoodSolver.launchDeg(distance, rpm, MIN_DEG, MAX_DEG))) return false;
        }
        return true;
    }

    @Test
    public void passRoundTrips() {
        double[] distances = {24, 48, 72, 96, 120, 144, 165};
        double[] angles = {45, 55, 65, 70};
        // Once with the speed the same at every angle, once with it changing.
        double[][][] multipliers = {exitSpeedAngle, {{40, 0.85}, {75, 1.15}}};
        for (double[][] multiplier : multipliers) {
            ShotTables.COAX_EXIT_SPEED_ANGLE = multiplier;
            for (double distance : distances) {
                for (double angle : angles) {
                    double rpm = HoodSolver.passRpm(distance, angle);
                    String where = distance + " in. at " + angle + "°: " + rpm + " RPM";
                    assertTrue(where, rpm > 0);
                    // It lands right there.
                    double landing = HoodSolver.heightAt(distance, angle, HoodSolver.exitSpeed(rpm, angle));
                    assertEquals(where, HoodSolver.PASS_LANDING_HEIGHT, landing, 1e-6);
                    assertEquals(where, angle, HoodSolver.passLaunchDeg(distance, rpm, angle, MIN_DEG, MAX_DEG), 0.01);
                }
            }
        }
    }

    @Test
    public void passKeepsTheArcNearerPreferDeg() {
        // At this RPM both arcs land at 120 in.: 60° and a flat one near 26°.
        double rpm = HoodSolver.passRpm(120, 60);
        double steep = HoodSolver.passLaunchDeg(120, rpm, 70, 20, 80);
        double flat = HoodSolver.passLaunchDeg(120, rpm, 30, 20, 80);
        assertEquals(60, steep, 0.01);
        assertTrue(flat + "°", flat < 40);
    }

    @Test
    public void passIsNaNWhenImpossible() {
        assertTrue(Double.isNaN(HoodSolver.passRpm(100, 90)));
        // Aimed into the floor.
        assertTrue(Double.isNaN(HoodSolver.passRpm(100, -60)));
        assertTrue(Double.isNaN(HoodSolver.passRpm(0, 45)));
        assertTrue(Double.isNaN(HoodSolver.passRpm(Double.NaN, 45)));
        assertTrue(Double.isNaN(HoodSolver.passRpm(100, Double.NaN)));
        // 500 RPM can't throw a ball 165 in.
        assertTrue(Double.isNaN(HoodSolver.passLaunchDeg(165, 500, 45, MIN_DEG, MAX_DEG)));
        assertTrue(Double.isNaN(HoodSolver.passLaunchDeg(100, 0, 45, MIN_DEG, MAX_DEG)));
        assertTrue(Double.isNaN(HoodSolver.passLaunchDeg(100, 2000, 45, MAX_DEG, MIN_DEG)));
        assertTrue(Double.isNaN(HoodSolver.passLaunchDeg(100, 2000, Double.NaN, MIN_DEG, MAX_DEG)));
        assertTrue(Double.isNaN(HoodSolver.passLaunchDeg(0, 2000, 45, MIN_DEG, MAX_DEG)));
    }

    @Test
    public void passLandsWithinTheTolerance() {
        double tolerance = HoodSolver.PASS_LANDING_TOLERANCE;
        int tried = 0, found = 0;
        for (double distance = 24; distance <= 165; distance += 12) {
            for (double prefer = 45; prefer <= 70; prefer += 25) {
                double base = HoodSolver.passRpm(distance, prefer);
                for (double rpm = base - 300; rpm <= base + 300; rpm += 25) {
                    double angle = HoodSolver.passLaunchDeg(distance, rpm, prefer, MIN_DEG, MAX_DEG);
                    tried++;
                    if (Double.isNaN(angle)) continue;
                    found++;
                    String where = distance + " in. at " + rpm + " RPM: " + angle + "°";
                    assertTrue(where, angle >= MIN_DEG && angle <= MAX_DEG);
                    // Still above the landing height before the window, below it after: it lands inside.
                    double speed = HoodSolver.exitSpeed(rpm, angle);
                    assertTrue(where, HoodSolver.heightAt(distance - tolerance, angle, speed) >= HoodSolver.PASS_LANDING_HEIGHT - EPS);
                    assertTrue(where, HoodSolver.heightAt(distance + tolerance, angle, speed) <= HoodSolver.PASS_LANDING_HEIGHT + EPS);
                }
            }
        }
        assertTrue("found " + found + " of " + tried, found > tried / 4);
    }

    @Test
    public void exitSpeedInterpolatesTheTableByRpm() {
        ShotTables.COAX_EXIT_SPEED = new double[][]{{2000, 0.10}, {4000, 0.14}};
        assertEquals(3000 * 0.12, HoodSolver.exitSpeed(3000), EPS);
        // Uses |rpm|.
        assertEquals(3000 * 0.12, HoodSolver.exitSpeed(-3000), EPS);
        // Past either end: the end's value.
        assertEquals(1000 * 0.10, HoodSolver.exitSpeed(1000), EPS);
        assertEquals(5000 * 0.14, HoodSolver.exitSpeed(5000), EPS);
        assertEquals(0, HoodSolver.exitSpeed(0), EPS);
    }

    @Test
    public void exitSpeedAtAnAngleUsesTheAngleMultiplier() {
        ShotTables.COAX_EXIT_SPEED_ANGLE = new double[][]{{40, 0.9}, {80, 1.1}};
        double base = HoodSolver.exitSpeed(3000);
        assertEquals(base * 1.0, HoodSolver.exitSpeed(3000, 60), EPS);
        assertEquals(base * 0.9, HoodSolver.exitSpeed(3000, 40), EPS);
        assertEquals(base * 0.9, HoodSolver.exitSpeed(3000, 20), EPS);
        assertEquals(base * 1.1, HoodSolver.exitSpeed(3000, 85), EPS);
        assertTrue(Double.isNaN(HoodSolver.exitSpeed(3000, Double.NaN)));
    }

    @Test
    public void orderProblemChecksTheCoaxTables() {
        assertNull(ShotTables.orderProblem());

        ShotTables.COAX_EXIT_SPEED = new double[][]{{5000, 0.12}, {2000, 0.12}};
        assertEquals("COAX_EXIT_SPEED RPMs are not increasing", ShotTables.orderProblem());
        ShotTables.COAX_EXIT_SPEED = exitSpeed;

        ShotTables.COAX_EXIT_SPEED_ANGLE = new double[][]{{75, 1.0}, {40, 1.0}};
        assertEquals("COAX_EXIT_SPEED_ANGLE angles are not increasing", ShotTables.orderProblem());
        ShotTables.COAX_EXIT_SPEED_ANGLE = exitSpeedAngle;

        ShotTables.COAX_RPM = new double[][]{{50, 3000}, {50, 3200}};
        assertEquals("COAX_RPM distances are not increasing", ShotTables.orderProblem());
    }
}
