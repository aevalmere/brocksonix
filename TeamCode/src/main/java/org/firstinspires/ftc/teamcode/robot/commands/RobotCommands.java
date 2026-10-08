package org.firstinspires.ftc.teamcode.robot.commands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.AtomicPath;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.curves.Line;

import org.firstinspires.ftc.teamcode.agateflow.AgateFlowCommands;
import org.firstinspires.ftc.teamcode.agateflow.DriveRoute;
import org.firstinspires.ftc.teamcode.agateflow.ParkGuard;
import org.firstinspires.ftc.teamcode.agateflow.Route;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotState;

import java.util.function.DoubleSupplier;

/**
 * Building blocks for autonomous. Each method returns an Ivy Command that drives,
 * shoots or intakes. Commands never touch motors or servos: they set what Robot
 * should do (the same fields the gamepads set) and wait on what Robot reports,
 * so auto and TeleOp share one copy of the aiming and firing logic.
 *
 * <pre>
 * RobotCommands commands = new RobotCommands(robot);
 * Scheduler.schedule(sequential(commands.driveTo(SHOOT_SPOT), commands.shootAll()));
 * while (opModeIsActive()) { Scheduler.execute(); robot.update(); }
 * </pre>
 *
 * The OpMode loop must call Scheduler.execute() and then robot.update() every
 * pass. Without robot.update() nothing moves.
 */
@Configurable
public class RobotCommands {
    // TODO(9): Time a normal 1-ball shot, then add margin. Long enough that a good shot never times out, short enough that a misfeed doesn't eat the auto.
    public static double SHOOT_ONE_TIMEOUT_MS = 1500;
    // TODO(9): Time a full burst the same way.
    public static double SHOOT_ALL_TIMEOUT_MS = 3000;
    // TODO(9): Jam a ball on purpose and run shootAllWithRetry. Raise until the retry frees it, but balls must not come out of the intake.
    public static double RETRY_UNJAM_MS = 200;
    // TODO(9): Each retry can cost RETRY_UNJAM_MS + SHOOT_ALL_TIMEOUT_MS. Set to 0 if retries never fix anything.
    public static int SHOOT_RETRIES = 1;
    // TODO(14): Time how long the flower intake takes to come all the way up (film it). Set this a little longer.
    /** flowerUp() waits this long for the flower intake to get up, ms, so a drive after it starts with the intake up. */
    public static double FLOWER_RAISE_MS = 400;

    private final Robot robot;
    /** shoot() counts from here, so it ends on its own ball and not on an earlier one. */
    private int shotsAtStart = 0;

    public RobotCommands(Robot robot) {
        this.robot = robot;
    }

    // ---- Driving ----

    /**
     * Drives a straight line from wherever the robot is when the command starts to
     * the pose (written for red, flipped for blue). The heading turns evenly along
     * the way. Ends when the end of the line is reached, and the follower keeps
     * settling on that pose after that. It has no time limit, so use the other
     * driveTo if the robot could get stuck.
     */
    public Command driveTo(Pose redPose) {
        // Lazy, because the start point is only known when the command starts.
        return Commands.lazy(() -> straightLineTo(redPose)).requiring(robot.drive);
    }

    /** Same as driveTo, but gives up after timeoutMs and holds where it is. */
    public Command driveTo(Pose redPose, double timeoutMs) {
        return driveTo(redPose).raceWith(Commands.waitMs(timeoutMs));
    }

    /** Runs any Pedro path, already in this alliance's field coordinates. Ends when the end of the path is reached. */
    public Command follow(Path path) {
        return PedroCommands.follow(robot.drive.follower(), path).requiring(robot.drive);
    }

    /**
     * Runs an AgateFlow drive and moves the flower intake to match its plan: down on
     * the stretches its Route marks with flowerIntakeDown(), up on all the others.
     * AgateFlow plans those stretches with the intake reaching further ahead, so the
     * robot's real shape matches what it planned for. A route that ends with such a
     * stretch leaves the intake down at its last stop, to pick up there; raise it with
     * flowerUp() after. If the drive ends any other way (no plan, out of time,
     * cancelled), the intake goes up, since every other plan has it up.
     *
     * Before the next drive, run flowerUp() (as below): it raises the intake and waits
     * for it. That drive is planned with the intake up from its start. Inside one Route,
     * AgateFlow already plans the stretch after a down stretch with the intake still
     * reaching out for its first AgateFlow.FLOWER_INTAKE_RAISE_RUN inches.
     *
     * <pre>
     * DriveRoute toFlower = agateFlow.driveTo(new Route().to(FLOWER).approach(FLOWER_DEGREES, 12).flowerIntakeDown());
     * sequential(commands.withFlowerIntake(toFlower), commands.waitUntilFull(1500), commands.flowerUp())
     * </pre>
     */
    public Command withFlowerIntake(DriveRoute drive) {
        FlowerIntakeFollower follower = new FlowerIntakeFollower(drive);
        // Never ends by itself: the drive ends the group.
        Command flower = Command.build()
                .setStart(follower::start)
                .setExecute(follower::execute)
                .setEnd(follower::end)
                .requiring(robot.flowerIntake);
        return Groups.deadline(drive, flower);
    }

    /** Keeps the flower intake where the drive's current stretch wants it (see withFlowerIntake). */
    private final class FlowerIntakeFollower {
        private final DriveRoute drive;
        private boolean started = false;
        /** Whether a stretch was seen yet, and whether it had the intake down. */
        private boolean known = false, down = false;

        FlowerIntakeFollower(DriveRoute drive) {
            this.drive = drive;
        }

        void start() {
            started = true;
            known = false;
        }

        void execute() {
            // Until the first plan is ready nobody knows the stretch, so leave the intake as it is.
            if (drive.plan() == null) return;
            boolean wanted = drive.flowerIntakeDown();
            // Moved only when the stretch changes, so Robot can still raise it when storage fills.
            if (!known || wanted != down) robot.flowerIntake.setDown(wanted);
            known = true;
            down = wanted;
        }

        /** Safe to call twice, or without start: Ivy groups can end a command both ways. */
        void end(EndCondition endCondition) {
            if (started && drive.status() != DriveRoute.Status.ARRIVED) robot.flowerIntake.setDown(false);
            started = false;
        }
    }

    private Command straightLineTo(Pose redPose) {
        if (RobotState.alliance == null) {
            throw new IllegalStateException("No alliance picked, so driveTo can't flip the pose.");
        }
        Follower follower = robot.drive.follower();
        Pose from = follower.pose();
        Pose to = RobotState.alliance.fromRed(redPose);

        // Pedro refuses a zero-length line. Already at the spot: just hold the target pose.
        if (Math.hypot(to.x() - from.x(), to.y() - from.y()) < 0.01) {
            return PedroCommands.hold(follower, to);
        }
        Path line = new AtomicPath(new Line(from, to)).linear(from, to);
        return PedroCommands.follow(follower, line).setEnd(endCondition -> {
            // Timed out or cancelled: stop chasing the target and hold still.
            if (endCondition == EndCondition.INTERRUPTED) follower.hold(follower.pose());
        });
    }

    // ---- Shooting ----
    // Shooting commands require the shooter, so only one of them runs at a time.

    /** Fires one ball. Ends after the shot, right away if storage is empty, or after SHOOT_ONE_TIMEOUT_MS. */
    public Command shoot() {
        Command fire = Command.build()
                .setStart(() -> {
                    robot.singleBallBursts = true;
                    shotsAtStart = robot.shotsFired();
                })
                // Hold shoot until our ball is counted, then let go so a second ball isn't fed.
                .setExecute(() -> robot.shootHeld = robot.shotsFired() == shotsAtStart && !robot.storage.isEmpty())
                .setDone(() -> !robot.isFiring() && (robot.shotsFired() > shotsAtStart || robot.storage.isEmpty()))
                .setEnd(endCondition -> stopShooting())
                .requiring(robot.shooter);
        return fire.raceWith(Commands.waitMs(SHOOT_ONE_TIMEOUT_MS));
    }

    /**
     * Fires every stored ball. Ends when storage is empty and the last feed has
     * finished, or after SHOOT_ALL_TIMEOUT_MS.
     */
    public Command shootAll() {
        Command fire = Command.build()
                .setStart(() -> robot.singleBallBursts = true)
                // Once storage is empty, let go of shoot. FireControl still finishes the feed in progress.
                .setExecute(() -> robot.shootHeld = !robot.storage.isEmpty())
                .setDone(() -> robot.storage.isEmpty() && !robot.isFiring())
                .setEnd(endCondition -> stopShooting())
                .requiring(robot.shooter);
        return fire.raceWith(Commands.waitMs(SHOOT_ALL_TIMEOUT_MS));
    }

    /**
     * shootAll. If balls are still stored after it (a jam made it time out), runs the
     * rail backwards for RETRY_UNJAM_MS and tries shootAll again, up to SHOOT_RETRIES times.
     */
    public Command shootAllWithRetry() {
        return new Retry(
                shootAll(),
                Groups.sequential(unjam(RETRY_UNJAM_MS), shootAll()),
                robot.storage::isEmpty,
                SHOOT_RETRIES);
    }

    /**
     * Shoots whenever a ball is stored and never ends by itself, so run it alongside
     * a drive command (see sweepAndShoot). Shoot is let go whenever storage is
     * empty, so the rail can collect between shots.
     */
    public Command shootContinuous() {
        return Commands.infinite(() -> robot.shootHeld = !robot.storage.isEmpty())
                .setStart(() -> robot.singleBallBursts = true)
                .setEnd(endCondition -> stopShooting())
                .requiring(robot.shooter);
    }

    /** Runs the drive command with shootContinuous alongside, and ends when the drive ends. */
    public Command sweepAndShoot(Command drive) {
        return Groups.deadline(drive, shootContinuous());
    }

    /** Safe to call twice, or on a command that never started: Ivy groups can end a command both ways. */
    private void stopShooting() {
        robot.shootHeld = false;
        robot.singleBallBursts = false;
    }

    // ---- Parking ----

    /**
     * Makes sure auto ends parked, whatever is running then (see agateflow/ParkGuard).
     * It stops every command here (each requires one of these subsystems) and holds
     * the flower intake up for the whole park drive: AgateFlow sizes the robot with
     * it up, and at the park drive's priority nothing else can lower it.
     *
     * @param agateFlow   made with robot.drive as its drive requirement
     * @param secondsLeft seconds left in auto, like {@code () -> 30 - matchTime.seconds()}
     */
    public ParkGuard parkGuard(AgateFlowCommands agateFlow, Route parkRoute, DoubleSupplier secondsLeft) {
        Runnable up = () -> robot.flowerIntake.setDown(false);
        Command holdFlowerUp = Commands.infinite(up).setStart(up).requiring(robot.flowerIntake);
        return new ParkGuard(agateFlow, parkRoute, secondsLeft, robot.shooter, robot.rail, robot.flowerIntake)
                .whileParking(holdFlowerUp);
    }

    // ---- Intake and rail ----

    public Command flowerDown() {
        return Commands.instant(() -> robot.flowerIntake.setDown(true)).requiring(robot.flowerIntake);
    }

    /**
     * Raises the flower intake and waits FLOWER_RAISE_MS for it to get up, so a drive
     * started after this has the intake up, as AgateFlow plans it.
     */
    public Command flowerUp() {
        return Groups.sequential(
                Commands.instant(() -> robot.flowerIntake.setDown(false)).requiring(robot.flowerIntake),
                Commands.waitMs(FLOWER_RAISE_MS));
    }

    /** Waits until storage is full or timeoutMs passes. The rail collects on its own while this waits. */
    public Command waitUntilFull(double timeoutMs) {
        return Commands.waitUntil(robot.storage::isFull).raceWith(Commands.waitMs(timeoutMs));
    }

    /** Runs the rail and intake backwards for ms, then lets go, even if the command is cancelled. */
    public Command unjam(double ms) {
        // This never finishes by itself. The race ends it when the wait is over.
        Command hold = Command.build()
                .setStart(() -> robot.unjamHeld = true)
                .setEnd(endCondition -> robot.unjamHeld = false);
        return hold.raceWith(Commands.waitMs(ms)).requiring(robot.rail);
    }
}
