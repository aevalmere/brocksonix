package org.firstinspires.ftc.teamcode.robot.shot;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.robot.subsystems.CoaxHood;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/**
 * ShotSolver with the coax hood: passes from the physics (valid only inside
 * PASS_RPM's distances, no hood angle for a wheel that's too slow, nothing from a
 * bad Panels number), and LEAD's flight time from the physics, falling back to
 * SHOT_FLIGHT_TIME when no hood angle scores.
 */
public class ShotSolverCoaxTest {
    private static final double EPS = 1e-9;
    private static final double MIN_DEG = 40;
    private static final double MAX_DEG = 75;
    // 60 in. straight ahead along x.
    private static final Pose ROBOT = new Pose(20, 30, 0);
    private static final Pose TARGET = new Pose(80, 30);
    // Sideways, so LEAD moves the aim point along y only.
    private static final Velocity SIDEWAYS = new Velocity(0, 20, 0);

    private boolean lead, predict;
    private int leadIterations;
    private double minDeg, maxDeg, passDeg, passLandingHeight;
    private double[][] shotFlightTime, passRpm, passFeedPower, exitSpeed, exitSpeedAngle;

    @Before
    public void remember() {
        lead = ShotSolver.LEAD;
        predict = ShotSolver.PREDICT;
        leadIterations = ShotSolver.LEAD_ITERATIONS;
        minDeg = CoaxHood.MIN_LAUNCH_DEG;
        maxDeg = CoaxHood.MAX_LAUNCH_DEG;
        passDeg = CoaxHood.PASS_LAUNCH_DEG;
        passLandingHeight = HoodSolver.PASS_LANDING_HEIGHT;
        shotFlightTime = ShotTables.SHOT_FLIGHT_TIME;
        passRpm = ShotTables.PASS_RPM;
        passFeedPower = ShotTables.PASS_FEED_POWER;
        exitSpeed = ShotTables.COAX_EXIT_SPEED;
        exitSpeedAngle = ShotTables.COAX_EXIT_SPEED_ANGLE;

        // The defaults, in case they were tuned, and our own tables.
        ShotSolver.LEAD = false;
        ShotSolver.PREDICT = false;
        ShotSolver.LEAD_ITERATIONS = 3;
        CoaxHood.MIN_LAUNCH_DEG = MIN_DEG;
        CoaxHood.MAX_LAUNCH_DEG = MAX_DEG;
        CoaxHood.PASS_LAUNCH_DEG = 60;
        HoodSolver.PASS_LANDING_HEIGHT = 1.4;
        ShotTables.SHOT_FLIGHT_TIME = new double[][]{{20, 0.5}, {120, 0.5}};
        ShotTables.PASS_RPM = new double[][]{{24, 1500}, {165, 3200}};
        ShotTables.PASS_FEED_POWER = new double[][]{{24, 1.0}, {165, 1.0}};
        ShotTables.COAX_EXIT_SPEED = new double[][]{{2000, 0.12}, {5000, 0.12}};
        ShotTables.COAX_EXIT_SPEED_ANGLE = new double[][]{{40, 1.0}, {75, 1.0}};
    }

    @After
    public void putBack() {
        ShotSolver.LEAD = lead;
        ShotSolver.PREDICT = predict;
        ShotSolver.LEAD_ITERATIONS = leadIterations;
        CoaxHood.MIN_LAUNCH_DEG = minDeg;
        CoaxHood.MAX_LAUNCH_DEG = maxDeg;
        CoaxHood.PASS_LAUNCH_DEG = passDeg;
        HoodSolver.PASS_LANDING_HEIGHT = passLandingHeight;
        ShotTables.SHOT_FLIGHT_TIME = shotFlightTime;
        ShotTables.PASS_RPM = passRpm;
        ShotTables.PASS_FEED_POWER = passFeedPower;
        ShotTables.COAX_EXIT_SPEED = exitSpeed;
        ShotTables.COAX_EXIT_SPEED_ANGLE = exitSpeedAngle;
    }

    @Test
    public void aCoaxPassInRangeIsValidAndLobsAtThePassAngle() {
        double rpm = HoodSolver.passRpm(60, 60);
        assertFalse(Double.isNaN(rpm));
        ShotSolution pass = new ShotSolver().solveCoaxPass(ROBOT, TARGET, rpm);
        assertTrue(pass.valid);
        assertEquals(60, pass.distance, EPS);
        assertEquals(0, pass.turretDeg, EPS);
        assertEquals(rpm, pass.rpm, EPS);
        // With the wheel at exactly that RPM, the angle that lands it is the lob angle itself.
        assertEquals(60, pass.launchDeg, 0.1);
    }

    @Test
    public void aCoaxPassOutsideThePassTableIsNotValid() {
        ShotSolver solver = new ShotSolver();
        // 10 in. away: under PASS_RPM's first distance (24).
        assertFalse(solver.solveCoaxPass(ROBOT, new Pose(30, 30), 1500).valid);
        // 170 in. away: past its last (165), where a pass could leave the field (G405).
        assertFalse(solver.solveCoaxPass(ROBOT, new Pose(20, 200), 3000).valid);
    }

    @Test
    public void aCoaxPassWithTheWheelTooSlowHasNoHoodAngle() {
        ShotSolution pass = new ShotSolver().solveCoaxPass(ROBOT, TARGET, 300);
        // In range, so the wheel is told the pass RPM, but nothing lands at 300 RPM yet,
        // so Robot doesn't feed (it needs hoodCanScore()).
        assertTrue(pass.valid);
        assertFalse(pass.hoodCanScore());
    }

    @Test
    public void aBadPanelsNumberGivesNoPass() {
        HoodSolver.PASS_LANDING_HEIGHT = Double.NaN;
        ShotSolution pass = new ShotSolver().solveCoaxPass(ROBOT, TARGET, 1500);
        assertFalse(pass.valid);
        assertEquals(0, pass.rpm, EPS);
    }

    @Test
    public void coaxLeadUsesThePhysicsFlightTime() {
        ShotSolver.LEAD = true;
        // Far off, so a lead from the table would show (2 s is 40 in. of lead).
        ShotTables.SHOT_FLIGHT_TIME = new double[][]{{20, 2.0}, {120, 2.0}};
        // An RPM where the hood scores from 60 in., with room either way.
        double rpm = HoodSolver.seedRpm(60, MIN_DEG, MAX_DEG, 5500);
        assertFalse(Double.isNaN(rpm));

        ShotSolution shot = new ShotSolver().solve(ROBOT, SIDEWAYS, TARGET, true, rpm, 0);
        assertTrue(shot.hoodCanScore());
        // The aim point, back from the turret angle and distance: LEAD moved it along y only.
        double aimY = 30 + shot.distance * Math.sin(Math.toRadians(shot.turretDeg));
        double leadInches = 30 - aimY;
        double flightTime = HoodSolver.flightTime(shot.distance, rpm,
                HoodSolver.launchDeg(shot.distance, rpm, MIN_DEG, MAX_DEG));
        assertEquals(20 * flightTime, leadInches, 0.2);
        assertTrue("lead " + leadInches, leadInches < 20);
    }

    @Test
    public void coaxLeadFallsBackToTheTableWhenNoAngleScores() {
        ShotSolver.LEAD = true;
        // Nothing reaches the cell at 500 RPM, so the table's 0.5 s: 10 in. of lead.
        ShotSolution shot = new ShotSolver().solve(ROBOT, SIDEWAYS, TARGET, true, 500, 0);
        assertFalse(shot.hoodCanScore());
        assertEquals(Math.hypot(60, 10), shot.distance, EPS);
        assertEquals(60, shot.targetDistance, EPS);
    }
}
