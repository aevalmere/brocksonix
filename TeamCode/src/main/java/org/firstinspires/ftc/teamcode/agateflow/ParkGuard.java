package org.firstinspires.ftc.teamcode.agateflow;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.math.Pose;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.function.DoubleSupplier;

/**
 * Makes sure auto ends parked. It keeps an up-to-date estimate (in the
 * background, every REFRESH_MS) of how long parking takes from wherever the
 * robot is right now. When the time left gets down to that estimate plus
 * MARGIN_SECONDS, it stops whatever is running and drives the park route.
 *
 * "Stops whatever is running": the park command requires the drive and every
 * subsystem passed in, at a high priority, so Ivy interrupts every command
 * using any of them (their end() runs, so shoot buttons and the like are let
 * go). Pass every subsystem your auto commands require.
 *
 * For this robot, make it with RobotCommands.parkGuard, which passes every
 * subsystem and keeps the flower intake up while parking:
 * <pre>
 * ElapsedTime matchTime = new ElapsedTime();   // reset when auto starts
 * ParkGuard park = commands.parkGuard(agateFlow, PARK_ROUTE, () -> 30 - matchTime.seconds());
 * while (opModeIsActive()) {
 *     park.update();         // before Scheduler.execute(), so parking starts this loop
 *     Scheduler.execute();
 *     robot.update();
 * }
 * </pre>
 * Not used by any OpMode yet.
 */
@Configurable
public final class ParkGuard {
    // TODO(15): Run an auto that runs long and check it ends parked with a little time to spare. Raise if it's late, lower if it parks too early.
    /** Start parking this much earlier than the estimate says it's needed, seconds. */
    public static double MARGIN_SECONDS = 0.75;
    /** How often the park estimate is redone from the robot's current pose, ms. */
    public static double REFRESH_MS = 250;
    /** Used only if no park estimate has worked yet (say the park route is blocked): park this many seconds before the end. */
    public static double FALLBACK_SECONDS = 5;
    /** Priority of the park command, above anything else the auto schedules. */
    public static int PRIORITY = 1000;

    public enum State { WATCHING, PARKING, DISARMED }

    private final AgateFlowCommands agateFlow;
    private final Route parkRoute;
    private final DoubleSupplier secondsLeft;
    private final Object[] stopAlso;
    private Command alongside;

    private State state = State.WATCHING;
    /** Newest estimate asked for, and the newest one that came back with a plan. */
    private Estimate asked, latest;
    private DriveRoute drive;
    private Command parking;

    /**
     * @param secondsLeft seconds left in auto, like {@code () -> 30 - matchTime.seconds()}
     * @param stopAlso    every subsystem the auto's commands require (the drive is added on its own)
     */
    public ParkGuard(AgateFlowCommands agateFlow, Route parkRoute, DoubleSupplier secondsLeft, Object... stopAlso) {
        this.agateFlow = agateFlow;
        this.parkRoute = parkRoute;
        this.secondsLeft = secondsLeft;
        this.stopAlso = stopAlso;
    }

    /** A command to run alongside the park drive. It is ended when the drive ends (so it may run forever, like holding a mechanism in place). */
    public ParkGuard whileParking(Command command) {
        alongside = command;
        return this;
    }

    /** Stop watching, say because the auto parked on its own. */
    public void disarm() {
        state = State.DISARMED;
    }

    public State state() {
        return state;
    }

    /** Seconds parking takes from here (calibrated, without the margin), or NaN before the first estimate. */
    public double parkSeconds() {
        return latest == null ? Double.NaN : latest.seconds();
    }

    /** Call every loop, before Scheduler.execute(). */
    public void update() {
        if (state == State.DISARMED) return;
        if (state == State.PARKING) {
            // The park drive gave up (no plan yet, or timed out) with time left: try again.
            if (drive.status() == DriveRoute.Status.NO_PLAN || drive.status() == DriveRoute.Status.TIMED_OUT) {
                if (!Scheduler.isScheduled(parking) && secondsLeft.getAsDouble() > 0.5) startParking(null);
            }
            return;
        }

        if (asked != null && asked.ready()) {
            if (asked.plan() != null && asked.plan().ok()) latest = asked;
            if (asked.age() * 1000 >= REFRESH_MS) asked = null;
        }
        if (asked == null) asked = agateFlow.estimate(parkRoute);

        if (secondsLeft.getAsDouble() <= needed()) startParking(fresh(latest));
    }

    /**
     * Estimated park time, plus the margin and the time a fresh plan takes if the
     * robot has moved. If the newer estimates keep failing (moving fast right next to
     * something, say), the last good one gets older, and counts as that much slower.
     */
    private double needed() {
        if (latest == null) return FALLBACK_SECONDS;
        Plan plan = latest.plan();
        if (atParkSpot(plan)) return Double.NEGATIVE_INFINITY;
        double stale = Math.max(0, latest.age() - REFRESH_MS / 1000);
        return latest.seconds() + plan.planMillis / 1000 + MARGIN_SECONDS + stale;
    }

    /** The estimate, if the robot is still where it was made (so the drive can start with it at once). */
    private static Estimate fresh(Estimate estimate) {
        return estimate != null && estimate.age() * 1000 < REFRESH_MS * 2 ? estimate : null;
    }

    private boolean atParkSpot(Plan plan) {
        Pose end = plan.legs.get(plan.legs.size() - 1).end;
        Pose pose = agateFlow.follower.pose();
        return Geometry.length(pose.x() - end.x(), pose.y() - end.y()) < 2;
    }

    private void startParking(Estimate estimate) {
        drive = estimate != null ? agateFlow.driveTo(estimate) : agateFlow.driveTo(parkRoute);
        CommandBuilder command = alongside == null ? drive : Groups.deadline(drive, alongside);
        Set<Object> everything = new HashSet<>(Arrays.asList(stopAlso));
        everything.add(agateFlow.drive);
        if (alongside != null) everything.addAll(alongside.requirements());
        command.requiring(everything);
        command.setPriority(PRIORITY);
        parking = command;
        Scheduler.schedule(parking);
        state = State.PARKING;
    }

    /** One line for telemetry. */
    public String describe() {
        switch (state) {
            case PARKING:
                return "Parking: " + drive.describe();
            case DISARMED:
                return "Park guard off";
            default:
                double left = secondsLeft.getAsDouble();
                return latest == null ? "Park: estimating"
                        : String.format("Park starts in %.1f s (takes %.1f s)", left - needed(), parkSeconds());
        }
    }
}
