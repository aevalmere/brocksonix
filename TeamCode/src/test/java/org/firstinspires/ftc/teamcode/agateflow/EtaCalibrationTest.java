package org.firstinspires.ftc.teamcode.agateflow;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotState;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

/**
 * What EtaCalibration learns from. record() itself: the filter, and the legs it
 * ignores. And which legs DriveRoute reports: only one that ran start to finish on
 * its own plan and got to its stop. Not the first leg of a replan, one that gave up
 * settling, or one of driveTo(estimate) the robot joined partway.
 */
public class EtaCalibrationTest {
    private static final Pose START = new Pose(24, 16, Math.PI / 2);
    private static final Pose STOP = new Pose(24, 60, Math.PI / 2);
    private static final Pose END = new Pose(24, 124, Math.PI / 2);
    /** TIME_SCALE before a drive: far from what any leg measures, so every report shows. */
    private static final double UNLEARNED = 1.6;

    private TestRobot.Saved saved;
    private TestRobot robot;
    private AgateFlowCommands agateFlow;

    @Before
    public void setUp() {
        saved = TestRobot.save();
        RobotState.alliance = Alliance.RED;
        robot = new TestRobot();
        robot.useVirtualClock();
        robot.place(START);
        agateFlow = new AgateFlowCommands(robot.follower, () -> TestRobot.VOLTS, new Object());
    }

    @After
    public void putBack() {
        saved.restore();
    }

    @Test
    public void recordMovesTimeScaleByTheFilter() {
        EtaCalibration.LEARN = true;
        EtaCalibration.TIME_SCALE = 1.0;
        EtaCalibration.LEARN_FILTER = 0.8;
        EtaCalibration.record(2.0, 3.0);
        assertEquals(1.0 + 0.2 * (1.5 - 1.0), EtaCalibration.TIME_SCALE, 1e-9);
        // 0 = no smoothing: straight to the latest real/predicted.
        EtaCalibration.LEARN_FILTER = 0;
        EtaCalibration.record(2.0, 2.5);
        assertEquals(1.25, EtaCalibration.TIME_SCALE, 1e-9);
    }

    @Test
    public void recordIgnoresLegsItCantLearnFrom() {
        EtaCalibration.TIME_SCALE = 1.0;
        EtaCalibration.LEARN_FILTER = 0;
        EtaCalibration.LEARN = false;
        EtaCalibration.record(2.0, 3.0);
        assertEquals(1.0, EtaCalibration.TIME_SCALE, 0);
        EtaCalibration.LEARN = true;
        // Too short to trust, way off (bumped or stuck), or no time at all.
        EtaCalibration.record(EtaCalibration.MIN_LEARN_SECONDS / 2, EtaCalibration.MIN_LEARN_SECONDS);
        EtaCalibration.record(2.0, 4.5);
        EtaCalibration.record(2.0, 0.9);
        EtaCalibration.record(2.0, 0);
        assertEquals(1.0, EtaCalibration.TIME_SCALE, 0);
    }

    /** Learning straight from each report (no smoothing), from UNLEARNED. */
    private static void learnEachLegAsIs() {
        EtaCalibration.LEARN = true;
        EtaCalibration.LEARN_FILTER = 0;
        EtaCalibration.TIME_SCALE = UNLEARNED;
    }

    /**
     * Runs the drive until it ends, calling {@code each} before every loop. Returns the
     * leg index (of the plan driven then) at each report to EtaCalibration: with no
     * smoothing, every report changes TIME_SCALE.
     */
    private List<Integer> reports(DriveRoute drive, Runnable each) {
        List<Integer> legs = new ArrayList<>();
        if (!Scheduler.isScheduled(drive)) Scheduler.schedule(drive);
        double scale = EtaCalibration.TIME_SCALE;
        int leg = 0;
        for (int i = 0; i < 3000 && Scheduler.isScheduled(drive); i++) {
            if (each != null) each.run();
            leg = drive.legIndex();
            robot.loop();
            if (EtaCalibration.TIME_SCALE != scale) legs.add(leg);
            scale = EtaCalibration.TIME_SCALE;
        }
        assertEquals(drive.problem(), DriveRoute.Status.ARRIVED, drive.status());
        return legs;
    }

    @Test
    public void everyLegOfACleanDriveIsLearned() {
        learnEachLegAsIs();
        List<Integer> legs = reports(agateFlow.driveTo(new Route().to(STOP).to(END)), null);
        assertEquals("[0, 1]", legs.toString());
        // The model matches the simulated robot within a few %.
        assertEquals(1.0, EtaCalibration.TIME_SCALE, 0.1);
    }

    @Test
    public void aReplannedPlanLearnsItsLaterLegsOnly() {
        learnEachLegAsIs();
        final DriveRoute drive = agateFlow.driveTo(new Route().to(STOP).to(END));
        final boolean[] pushed = {false};
        List<Integer> legs = reports(drive, () -> {
            // Pushed sideways on the way to the first stop: the leg it was on and the replan's
            // first leg say nothing about the estimate. The replan's second leg does.
            if (!pushed[0] && drive.plan() != null && robot.pose().y() > 30) {
                robot.place(new Pose(36, robot.pose().y(), Math.PI / 2));
                pushed[0] = true;
            }
        });
        assertTrue(pushed[0]);
        assertEquals("[1]", legs.toString());
    }

    @Test
    public void aLegThatGivesUpSettlingIsNotLearned() {
        learnEachLegAsIs();
        final DriveRoute drive = agateFlow.driveTo(new Route().to(END));
        List<Integer> legs = reports(drive, () -> {
            // Once Foresight is holding the end, something keeps the robot 2 in. short of it.
            if (drive.plan() != null && drive.legIndex() == 0 && !robot.follower.following() && robot.pose().y() > END.y() - 10) {
                robot.place(new Pose(END.x(), END.y() - 2, END.heading()));
            }
        });
        assertEquals("[]", legs.toString());
        assertEquals(UNLEARNED, EtaCalibration.TIME_SCALE, 0);
    }

    @Test
    public void aDriveJoinedPartwayLearnsOnlyItsLaterLegs() {
        learnEachLegAsIs();
        Estimate estimate = agateFlow.estimate(new Route().to(STOP).to(END));
        TestRobot.waitForPlans();
        assertTrue(estimate.plan().ok());
        // The robot sets off along the estimate's first leg before the drive starts (driving it
        // by hand, say), then the drive joins it there, on its predicted path.
        robot.follower.follow(estimate.plan().legs.get(0).path);
        for (int i = 0; i < 25; i++) robot.step();
        assertTrue(estimate.age() < 0.5);
        DriveRoute drive = agateFlow.driveTo(estimate);
        Scheduler.schedule(drive);
        robot.loop();
        assertTrue("used the estimate's plan", drive.plan() == estimate.plan());
        assertEquals("[1]", reports(drive, null).toString());
    }
}
