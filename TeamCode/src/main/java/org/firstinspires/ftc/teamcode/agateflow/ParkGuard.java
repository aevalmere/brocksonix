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
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Makes sure auto ends parked. It keeps an up-to-date estimate (in the
 * background, every REFRESH_MS) of how long parking takes from wherever the
 * robot is right now. When the time left gets down to that estimate plus
 * MARGIN_SECONDS, it stops whatever is running and drives the park route.
 * If the park drive ends without getting there (no plan, out of time,
 * interrupted, or blocked from starting), it starts it again while there is
 * time left.
 *
 * "Stops whatever is running": the park command requires the drive and every
 * subsystem passed in, at a high priority, so Ivy interrupts every command
 * using any of them (their end() runs, so shoot buttons and the like are let
 * go). Pass every subsystem your auto commands require.
 *
 * The estimates share AgateFlow's one planning thread with the drives, so none
 * is asked for while a drive's own plan is being made: that one goes first.
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
    // TODO(15): Run an auto that replans a lot. Raise this if replans come back late (they share the planning thread with park estimates), lower it if parking starts much earlier than needed.
    /** How often the park estimate is redone from the robot's current pose, ms. */
    public static double REFRESH_MS = 250;
    // TODO(15): Block the park route with a vision zone in a test auto and check the robot still parks in time once it is clear.
    /** Used only if no park estimate has worked yet (say the park route is blocked): park this many seconds before the end. */
    public static double FALLBACK_SECONDS = 5;
    /** Priority of the park command, above anything else the auto schedules. */
    public static int PRIORITY = 1000;

    public enum State { WATCHING, PARKING }

    private final AgateFlowCommands agateFlow;
    private final Route parkRoute;
    private final DoubleSupplier secondsLeft;
    private final Object[] stopAlso;
    private Command alongside;
    private BooleanSupplier flowerIntakeDown = () -> false;

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

    /**
     * Where to read whether the flower intake is down (like robot.flowerIntake::isDown).
     * Parking from a FLOWER with it down, the park drive first backs out with the
     * intake still reaching ahead while it comes up, like after a
     * Route.flowerIntakeDown stop. Without this it plans with the intake up.
     */
    public ParkGuard flowerIntakeDown(BooleanSupplier isDown) {
        flowerIntakeDown = isDown;
        return this;
    }

    public State state() {
        return state;
    }

    /** Seconds parking takes from here (calibrated, without the margin), or NaN before the first estimate. */
    public double parkSeconds() {
        return latest == null ? Double.NaN : latest.seconds();
    }

    /** The park drive, null before parking starts. For the tests. */
    DriveRoute parkDrive() {
        return drive;
    }

    /** Call every loop, before Scheduler.execute(). */
    public void update() {
        if (state == State.PARKING) {
            // The park drive ended without getting there (no plan, out of time, interrupted), or was never
            // let start (Ivy drops a command blocked by a higher priority one): try again while there's time.
            boolean gaveUp = !Scheduler.isScheduled(parking) && drive.status() != DriveRoute.Status.ARRIVED;
            if (gaveUp && secondsLeft.getAsDouble() > 0.5) startParking(null);
            return;
        }

        if (asked != null && asked.ready()) {
            if (asked.plan() != null && asked.plan().ok()) latest = asked;
            if (asked.age() * 1000 >= REFRESH_MS) asked = null;
        }
        // A drive's own plan goes first on the planning thread: estimate again once it is made.
        if (asked == null && !agateFlow.drivePlanPending()) asked = agateFlow.estimate(parkRouteNow());

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

    /** The park route, starting with the flower intake as it is right now. */
    private Route parkRouteNow() {
        return parkRoute.startingFlowerDown(flowerIntakeDown.getAsBoolean());
    }

    private void startParking(Estimate estimate) {
        Route route = parkRouteNow();
        // An estimate made with the flower intake the other way planned the start wrong: plan again.
        boolean fits = estimate != null && estimate.route.startFlowerDown == route.startFlowerDown;
        drive = fits ? agateFlow.driveTo(estimate) : agateFlow.driveTo(route);
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
        if (state == State.PARKING) return "Parking: " + drive.describe();
        double left = secondsLeft.getAsDouble();
        return latest == null ? "Park: estimating"
                : String.format("Park starts in %.1f s (takes %.1f s)", left - needed(), parkSeconds());
    }
}
