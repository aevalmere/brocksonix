package org.firstinspires.ftc.teamcode.agateflow;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.FieldZones;
import org.firstinspires.ftc.teamcode.robot.RobotState;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.function.DoubleSupplier;

/**
 * Ivy commands that drive with AgateFlow: give a Route, get a Command that
 * plans from wherever the robot is when it starts, drives every leg, replans if
 * a zone appears in the way or the robot gets pushed off, and times itself out
 * from its own estimate (see DriveRoute). Not used by any OpMode yet.
 *
 * <pre>
 * AgateFlowCommands agateFlow = new AgateFlowCommands(robot.drive.follower(), robot.voltage::get, robot.drive);
 * DriveRoute toShot = agateFlow.driveTo(new Route().to(SHOOT_SPOT).heading(Heading.finishBy(0.5)));
 * Scheduler.schedule(sequential(toShot, commands.shootAllWithRetry()));
 * </pre>
 *
 * Planning runs on one background thread, so the loop never waits for it. Until
 * the first plan is ready the robot stays as it was (holding, or finishing the
 * last path). To have nothing to wait for at the start of auto, call
 * DriveRoute.planFrom(startPose) in init.
 *
 * Vision: put what the camera sees in map() with map().setZones("vision", zones)
 * (safe from the camera thread), and give targets with Route.toLive(...). A
 * running DriveRoute replans when new zones block its path.
 */
@Configurable
public class AgateFlowCommands {
    // TODO(15): Watch a few autos: raise if a drive times out close to its target, lower if a stuck robot waits too long.
    /**
     * A route times out this many times its first estimate, plus TIMEOUT_EXTRA_SECONDS,
     * after its first plan is ready. Then the robot holds and the command ends.
     */
    public static double TIMEOUT_SCALE = 1.5;
    // TODO(15): Check together with TIMEOUT_SCALE on the robot.
    public static double TIMEOUT_EXTRA_SECONDS = 1.0;
    // TODO(15): Check on the robot with something pushing it off its path: the drive should still end.
    /**
     * A replan for a push can move the timeout later, but at most this many seconds
     * past the first one, so a route that keeps replanning still ends.
     */
    public static double REPLAN_EXTRA_SECONDS = 5.0;
    /**
     * A replan for a detour around something new in the way gets as long as its own
     * estimate says (TIMEOUT_SCALE x estimate + TIMEOUT_EXTRA_SECONDS), past the cap
     * above, but only this many times per drive, so a route that keeps finding new
     * blocks still ends.
     */
    public static int MAX_DETOURS = 3;
    /**
     * Past its deadline, a drive that is still getting closer to its stop goes on for
     * up to this many seconds more before it times out, seconds. A stuck one stops at once.
     */
    public static double MAX_OVERTIME_SECONDS = 3.0;
    // TODO(15): In a test OpMode, add a vision zone 15 in. in front of the robot while it drives at full speed. It should stop short and straight, and the battery reading shouldn't drop under 9 V. Lower this if the wheels skid or the hub browns out.
    /**
     * Reverse power, 0 to 1, that DriveRoute brakes with when something new shows up
     * close ahead, until the robot is nearly still (then it holds). It only slows the
     * robot down, and doesn't turn it. From 52 in/s (simulated) it stops in about 7 in.
     * at 0.5, 5 in. at 1 (as short as the wheels' grip allows), and 11 in. at 0.2
     * (Foresight's own maxBrakingPower, which a hold brakes with). More is shorter
     * but draws much more current: pushing backward while the motors still spin
     * forward adds their voltages, so by DriveModel's battery numbers the battery sags
     * to about 7.4 V at 1 (a brownout risk), 10.3 V at 0.5 and 11.8 V at 0.2.
     */
    public static double STOP_BRAKING_POWER = 0.5;
    /** Replan when the robot is this far from where it was predicted to be, inches (it got pushed). */
    public static double REPLAN_OFF_PATH = 8;
    /** Check for replanning at most this often, ms. */
    public static double REPLAN_EVERY_MS = 250;
    public static boolean REPLAN = true;

    /** One planning thread for the whole app, so OpModes starting and stopping don't pile up threads. */
    private static final ExecutorService WORKER = Executors.newSingleThreadExecutor(task -> {
        Thread thread = new Thread(task, "AgateFlow");
        thread.setDaemon(true);
        return thread;
    });

    final Follower follower;
    final Object drive;
    private final FieldMap map = new FieldMap();
    private final AgateFlow agateFlow;
    private final DoubleSupplier batteryVolts;

    /**
     * @param follower     the robot's follower
     * @param batteryVolts the hub's battery reading (a VoltageCache), read on the main thread
     * @param drive        the drive subsystem, so the commands require it
     */
    public AgateFlowCommands(Follower follower, DoubleSupplier batteryVolts, Object drive) {
        this.follower = follower;
        this.batteryVolts = batteryVolts;
        this.drive = drive;
        agateFlow = new AgateFlow(map, follower);
        // Made in init, while the follower is idle: a good time to read Foresight's gains (see DriveModel.readGains).
        DriveModel.capture(follower);
    }

    /** Zones to drive around. FieldZones are added on their own; add vision zones under another name. */
    public FieldMap map() {
        return map;
    }

    /** A command that drives the route. Poses are written for red, like everywhere else. */
    public DriveRoute driveTo(Route route) {
        return new DriveRoute(this, route);
    }

    /** A command that drives to one pose (written for red) and stops. */
    public DriveRoute driveTo(Pose redPose) {
        return driveTo(new Route().to(redPose));
    }

    /**
     * A command that drives an estimated route. It uses the estimate's plan and
     * starts at once if the robot is still where the estimate was made from
     * (within 1 in. and 5°), or if the estimate is under half a second old and the
     * robot is still on its predicted path. Otherwise it plans again from where the
     * robot is. Either way it replans while driving, like any DriveRoute.
     */
    public DriveRoute driveTo(Estimate estimate) {
        DriveRoute command = new DriveRoute(this, estimate.route);
        command.prepared = estimate;
        return command;
    }

    /**
     * How long the route would take from where the robot is now, worked out in the
     * background without driving it. Ready a few loops later (Estimate.ready()).
     */
    public Estimate estimate(Route route) {
        return estimateFrom(follower.pose(), follower.velocity(), route);
    }

    /** Same, from another pose (field coordinates), standing still. For planning in init. */
    public Estimate estimateFrom(Pose fieldStart, Route route) {
        return estimateFrom(fieldStart, new Velocity(0, 0, 0), route);
    }

    private Estimate estimateFrom(Pose start, Velocity velocity, Route route) {
        Alliance alliance = RobotState.alliance;
        return new Estimate(route, planLater(start, velocity, route, alliance), start, alliance);
    }

    /**
     * Plans on the background thread. Call on the main thread: it reads the
     * follower and the battery, and asks live waypoints for their poses here, so
     * their suppliers never run on the planning thread.
     */
    Future<Plan> planLater(final Pose start, final Velocity velocity, Route route, final Alliance alliance) {
        map.setZones("field", FieldZones.zones());
        final DriveModel settings = DriveModel.capture(follower);
        final double volts = batteryVolts.getAsDouble();
        final Route asked = route.askLivePosesNow();
        return WORKER.submit(() -> agateFlow.plan(start, velocity, asked, alliance, volts, settings));
    }
}
