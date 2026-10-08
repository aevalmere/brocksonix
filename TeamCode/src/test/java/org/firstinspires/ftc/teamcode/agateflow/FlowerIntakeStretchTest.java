package org.firstinspires.ftc.teamcode.agateflow;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Flower;
import org.firstinspires.ftc.teamcode.field.IntakePoses;
import org.firstinspires.ftc.teamcode.robot.RobotState;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

/**
 * DriveRoute.flowerIntakeDown(), which RobotCommands.withFlowerIntake follows: a
 * drive on the simulated robot, run like withFlowerIntake runs it (an Ivy deadline
 * group with the drive first), must say "down" exactly on the stretch the Route
 * marks with flowerIntakeDown(), and keep saying it after arriving there.
 */
public class FlowerIntakeStretchTest {
    private static final Pose SHOOT = new Pose(36, 48, Math.PI);
    private double timeLimit;
    private boolean replan;
    private Alliance alliance;

    @Before
    public void setUp() {
        timeLimit = AgateFlow.PLAN_TIME_LIMIT_MS;
        replan = AgateFlowCommands.REPLAN;
        alliance = RobotState.alliance;
        AgateFlow.PLAN_TIME_LIMIT_MS = 1e9;
        AgateFlowCommands.REPLAN = false;
        RobotState.alliance = Alliance.RED;
        Scheduler.reset();
    }

    @After
    public void putBack() {
        AgateFlow.PLAN_TIME_LIMIT_MS = timeLimit;
        AgateFlowCommands.REPLAN = replan;
        RobotState.alliance = alliance;
        Scheduler.reset();
    }

    /** One loop of the drive: where the robot was and what the drive said. */
    private static final class Sample {
        final double x, y;
        final boolean down;

        Sample(double x, double y, boolean down) {
            this.x = x;
            this.y = y;
            this.down = down;
        }
    }

    /** Drives the route from start like withFlowerIntake does, recording flowerIntakeDown() each loop the plan is known. */
    private static List<Sample> drive(Pose start, Route route, DriveRoute[] made) throws InterruptedException {
        final TestRobot robot = new TestRobot();
        robot.place(start);
        AgateFlowCommands agateFlow = new AgateFlowCommands(robot.follower, () -> TestRobot.VOLTS, new Object());
        final DriveRoute drive = agateFlow.driveTo(route);
        made[0] = drive;
        assertFalse(drive.flowerIntakeDown());
        final List<Sample> samples = new ArrayList<>();
        Command follow = Command.build().setExecute(() -> {
            if (drive.plan() != null) samples.add(new Sample(robot.pose().x(), robot.pose().y(), drive.flowerIntakeDown()));
        }).requiring(new Object());
        Command group = Groups.deadline(drive, follow);
        Scheduler.schedule(group);
        for (int i = 0; i < 20000 && Scheduler.isScheduled(group); i++) {
            Scheduler.execute();
            robot.step();
            // Planning runs on its own thread in real time: don't run the clock on while waiting.
            if (drive.status() == DriveRoute.Status.PLANNING) Thread.sleep(1);
        }
        assertEquals(drive.problem(), DriveRoute.Status.ARRIVED, drive.status());
        return samples;
    }

    @Test
    public void downOnlyOnTheMarkedStretch() throws InterruptedException {
        Pose through = new Pose(25, 80, 0);
        Pose flower = IntakePoses.flowerPose(Flower.ALLIANCE_WALL);
        DriveRoute[] drive = new DriveRoute[1];
        List<Sample> samples = drive(new Pose(30, 115, -Math.PI / 2), new Route()
                .through(through.x(), through.y())
                .to(flower).approach(IntakePoses.flowerTravelDegrees(Flower.ALLIANCE_WALL), IntakePoses.FLOWER_APPROACH).flowerIntakeDown()
                .to(SHOOT), drive);
        // Up, then down from the through-waypoint (same leg) to the FLOWER, then up again for the last leg.
        int changes = 0, lowered = -1, raised = -1;
        for (int i = 1; i < samples.size(); i++) {
            if (samples.get(i).down == samples.get(i - 1).down) continue;
            changes++;
            if (samples.get(i).down) lowered = i;
            else raised = i;
        }
        assertFalse(samples.get(0).down);
        assertEquals(2, changes);
        assertTrue(lowered > 0 && raised > lowered);
        Sample down = samples.get(lowered), up = samples.get(raised);
        assertTrue("lowered " + Math.hypot(down.x - through.x(), down.y - through.y()) + " in. from the through-waypoint",
                Math.hypot(down.x - through.x(), down.y - through.y()) < 6);
        assertTrue("raised " + Math.hypot(up.x - flower.x(), up.y - flower.y()) + " in. from the FLOWER stop",
                Math.hypot(up.x - flower.x(), up.y - flower.y()) < MotionModel.ARRIVED_INCHES + 0.5);
        assertFalse(samples.get(samples.size() - 1).down);
        assertFalse(drive[0].flowerIntakeDown());
    }

    @Test
    public void staysDownAfterArrivingOnAMarkedStretch() throws InterruptedException {
        DriveRoute[] drive = new DriveRoute[1];
        List<Sample> samples = drive(SHOOT, new Route().to(IntakePoses.flowerPose(Flower.ALLIANCE_WALL))
                .approach(IntakePoses.flowerTravelDegrees(Flower.ALLIANCE_WALL), IntakePoses.FLOWER_APPROACH).flowerIntakeDown(), drive);
        // Down from the start, and still down in the loop the drive arrived in (the group runs this after the drive).
        for (Sample sample : samples) assertTrue(sample.down);
        assertTrue(drive[0].flowerIntakeDown());
    }
}
