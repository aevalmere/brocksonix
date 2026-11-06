package org.firstinspires.ftc.teamcode.robot.shot;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.drivetrain.DrivePowers;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/** TurnPredictor: the first-order turn model, the lead it gives, and Pedro's wheel scaling. */
public class TurnPredictorTest {
    private static final double EPS = 1e-6;
    private static final double DT = 0.01;

    private boolean enabled;
    private double maxRate;
    private double tau;
    private double lead;
    private double response;

    @Before
    public void setUp() {
        response = TurnPredictor.TURRET_RESPONSE_SEC;
        TurnPredictor.TURRET_RESPONSE_SEC = 0.08;
        enabled = TurnPredictor.ENABLED;
        maxRate = TurnPredictor.MAX_TURN_RATE_DEG_PER_SEC;
        tau = TurnPredictor.TURN_TIME_CONSTANT_SEC;
        lead = TurnPredictor.LEAD_SEC;
        TurnPredictor.ENABLED = true;
        TurnPredictor.MAX_TURN_RATE_DEG_PER_SEC = 360;
        TurnPredictor.TURN_TIME_CONSTANT_SEC = 0.15;
        TurnPredictor.LEAD_SEC = 0.05;
    }

    @After
    public void putBack() {
        TurnPredictor.ENABLED = enabled;
        TurnPredictor.MAX_TURN_RATE_DEG_PER_SEC = maxRate;
        TurnPredictor.TURN_TIME_CONSTANT_SEC = tau;
        TurnPredictor.LEAD_SEC = lead;
        TurnPredictor.TURRET_RESPONSE_SEC = response;
    }

    @Test
    public void leadsTheMomentTheStickMoves() {
        TurnPredictor p = new TurnPredictor();
        p.update(new DrivePowers(0, 0, 1), DT);
        // The robot has barely started turning, but the prediction already leads.
        assertTrue(p.rateDegPerSec() < 30);
        assertTrue(p.rateAheadDegPerSec() > p.rateDegPerSec());
        assertTrue(p.headingAheadDeg() > 0);
    }

    @Test
    public void settlesOnFullRateAndLeadsByRateTimesLead() {
        TurnPredictor p = new TurnPredictor();
        for (int i = 0; i < 300; i++) p.update(new DrivePowers(0, 0, -0.5), DT);
        assertEquals(-180, p.rateDegPerSec(), EPS);
        assertEquals(-180, p.rateAheadDegPerSec(), EPS);
        assertEquals(-180 * 0.05, p.headingAheadDeg(), EPS);
    }

    @Test
    public void keepsTurningForAMomentAfterLettingGo() {
        TurnPredictor p = new TurnPredictor();
        for (int i = 0; i < 300; i++) p.update(new DrivePowers(0, 0, 1), DT);
        p.update(new DrivePowers(0, 0, 0), DT);
        assertTrue(p.rateDegPerSec() > 300);
        // Slowing down: less ahead than now, but still turning the same way.
        assertTrue(p.rateAheadDegPerSec() < p.rateDegPerSec());
        assertTrue(p.headingAheadDeg() > 0);
        assertTrue(p.headingAheadDeg() < p.rateDegPerSec() * 0.05);
    }

    @Test
    public void pushesExtraWhileTheTurnStartsAndNothingExtraOnceSteady() {
        TurnPredictor p = new TurnPredictor();
        p.update(new DrivePowers(0, 0, 1), DT);
        assertTrue("extra push at the start", p.feedforwardRateDegPerSec() > p.rateAheadDegPerSec() + 100);
        // A smaller stick jump gives a smaller extra push.
        TurnPredictor gentle = new TurnPredictor();
        gentle.update(new DrivePowers(0, 0, 0.3), DT);
        assertTrue(gentle.feedforwardRateDegPerSec() < p.feedforwardRateDegPerSec());
        for (int i = 0; i < 300; i++) p.update(new DrivePowers(0, 0, 1), DT);
        assertEquals("steady: just the robot's rate", 360, p.feedforwardRateDegPerSec(), EPS);
        // Letting go brakes the turret ahead of the robot too.
        p.update(new DrivePowers(0, 0, 0), DT);
        assertTrue(p.feedforwardRateDegPerSec() < p.rateAheadDegPerSec());
    }

    @Test
    public void offGivesNoLead() {
        TurnPredictor p = new TurnPredictor();
        TurnPredictor.ENABLED = false;
        p.update(new DrivePowers(0, 0, 1), DT);
        assertEquals(0, p.headingAheadDeg(), EPS);
        assertEquals(0, p.rateAheadDegPerSec(), EPS);
    }

    @Test
    public void drivingAtFullStickTurnsSlower() {
        // Full forward and full turn: a wheel would get 2, so all four are halved.
        assertEquals(0.5, TurnPredictor.effectiveTurn(new DrivePowers(1, 0, 1)), EPS);
        assertEquals(1, TurnPredictor.effectiveTurn(new DrivePowers(0, 0, 1)), EPS);
        assertEquals(-0.3, TurnPredictor.effectiveTurn(new DrivePowers(0.2, 0.2, -0.3)), EPS);
    }
}
