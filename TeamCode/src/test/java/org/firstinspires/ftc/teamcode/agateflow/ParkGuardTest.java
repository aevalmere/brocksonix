package org.firstinspires.ftc.teamcode.agateflow;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotState;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.function.DoubleSupplier;

/**
 * ParkGuard on the simulated robot, on a virtual clock: it starts parking when the
 * time left gets down to the park estimate plus the margin, interrupting what the
 * auto is doing, and gets there in time; and it starts the park drive again when it
 * finds no plan or is interrupted.
 */
public class ParkGuardTest {
    private static final Pose START = new Pose(24, 16, Math.PI / 2);
    private static final Pose PARK = new Pose(24, 60, Math.PI / 2);

    private TestRobot.Saved saved;
    private TestRobot robot;
    private final Object driveSubsystem = new Object(), shooter = new Object();
    private AgateFlowCommands agateFlow;
    private double startedAt;

    @Before
    public void setUp() {
        saved = TestRobot.save();
        RobotState.alliance = Alliance.RED;
        EtaCalibration.LEARN = false;
        robot = new TestRobot();
        robot.useVirtualClock();
        robot.place(START);
        agateFlow = new AgateFlowCommands(robot.follower, () -> TestRobot.VOLTS, driveSubsystem);
        startedAt = robot.seconds();
    }

    @After
    public void putBack() {
        saved.restore();
    }

    /** Seconds left in an auto that has {@code total} seconds from the start of the test. */
    private DoubleSupplier secondsLeft(final double total) {
        return () -> total - (robot.seconds() - startedAt);
    }

    /** One auto loop: the guard first, then the commands and the robot. */
    private void loop(ParkGuard park) {
        park.update();
        robot.loop();
    }

    @Test
    public void startsParkingWhenTheTimeLeftIsTheEstimatePlusTheMarginAndGetsThere() {
        // The auto is busy with something that never ends, using the drive and the shooter.
        final EndCondition[] autoEnded = {null};
        Command auto = Command.build().setEnd(how -> autoEnded[0] = how).requiring(driveSubsystem, shooter);
        Scheduler.schedule(auto);
        // Longer than FALLBACK_SECONDS, which counts until the first estimate is back.
        DoubleSupplier left = secondsLeft(ParkGuard.FALLBACK_SECONDS + 1);
        ParkGuard park = new ParkGuard(agateFlow, new Route().to(PARK), left, shooter);

        double leftBefore = Double.NaN;
        for (int i = 0; i < 1000 && park.state() == ParkGuard.State.WATCHING; i++) {
            leftBefore = left.getAsDouble();
            loop(park);
        }
        assertEquals(ParkGuard.State.PARKING, park.state());
        // What the auto was doing was interrupted, and the park drive started with the estimate's plan at once.
        assertEquals(EndCondition.INTERRUPTED, autoEnded[0]);
        DriveRoute drive = park.parkDrive();
        assertTrue(Scheduler.isScheduled(drive));
        assertTrue("drove the estimate's plan", drive.plan() != null);
        // It started in the loop the time left first got down to what it needs: the estimate, the
        // margin, and the estimate's own planning time (real milliseconds, so they vary a little).
        double needed = park.parkSeconds() + ParkGuard.MARGIN_SECONDS + drive.plan().planMillis / 1000;
        assertTrue(String.format("started with %.3f s left, needs %.3f s", leftBefore, needed),
                leftBefore <= needed + 0.05 && leftBefore > needed - 0.2);

        for (int i = 0; i < 1000 && drive.status() != DriveRoute.Status.ARRIVED; i++) loop(park);
        assertEquals(drive.problem(), DriveRoute.Status.ARRIVED, drive.status());
        assertTrue(Math.hypot(robot.pose().x() - PARK.x(), robot.pose().y() - PARK.y()) < 1.5);
        assertTrue("parked with " + left.getAsDouble() + " s left", left.getAsDouble() > 0);
    }

    @Test
    public void triesAgainAfterNoPlan() {
        // The park spot comes from vision, and isn't seen at first: no estimate, so it parks
        // FALLBACK_SECONDS before the end, and the first park drives find no plan.
        final boolean[] seen = {false};
        Route route = new Route().toLive(() -> seen[0] ? PARK : null);
        ParkGuard park = new ParkGuard(agateFlow, route, secondsLeft(ParkGuard.FALLBACK_SECONDS + 1), shooter);
        for (int i = 0; i < 300 && park.state() == ParkGuard.State.WATCHING; i++) loop(park);
        assertEquals(ParkGuard.State.PARKING, park.state());
        DriveRoute first = park.parkDrive();
        for (int i = 0; i < 10 && first.status() != DriveRoute.Status.NO_PLAN; i++) loop(park);
        assertEquals(DriveRoute.Status.NO_PLAN, first.status());

        seen[0] = true;
        for (int i = 0; i < 1000 && park.parkDrive().status() != DriveRoute.Status.ARRIVED; i++) loop(park);
        assertNotSame(first, park.parkDrive());
        assertEquals(DriveRoute.Status.ARRIVED, park.parkDrive().status());
        assertTrue(Math.hypot(robot.pose().x() - PARK.x(), robot.pose().y() - PARK.y()) < 1.5);
    }

    @Test
    public void triesAgainAfterBeingInterrupted() {
        ParkGuard.MARGIN_SECONDS = 5;
        ParkGuard park = new ParkGuard(agateFlow, new Route().to(PARK), secondsLeft(8), shooter);
        for (int i = 0; i < 400 && park.state() == ParkGuard.State.WATCHING; i++) loop(park);
        DriveRoute first = park.parkDrive();
        for (int i = 0; i < 100 && first.status() != DriveRoute.Status.DRIVING; i++) loop(park);
        for (int i = 0; i < 20; i++) loop(park);

        // Something even more important takes the drive for a few loops. Meanwhile the guard's tries
        // to start again are blocked (Ivy drops them), and it keeps trying.
        final int[] loops = {0};
        Command urgent = Command.build().setDone(() -> ++loops[0] > 5).requiring(driveSubsystem).setPriority(ParkGuard.PRIORITY + 1);
        Scheduler.schedule(urgent);
        assertEquals(DriveRoute.Status.STOPPED, first.status());
        for (int i = 0; i < 3; i++) loop(park);
        assertEquals(DriveRoute.Status.WAITING, park.parkDrive().status());

        for (int i = 0; i < 1000 && park.parkDrive().status() != DriveRoute.Status.ARRIVED; i++) loop(park);
        assertNotSame(first, park.parkDrive());
        assertEquals(park.parkDrive().problem(), DriveRoute.Status.ARRIVED, park.parkDrive().status());
        assertTrue(Math.hypot(robot.pose().x() - PARK.x(), robot.pose().y() - PARK.y()) < 1.5);
    }
}
