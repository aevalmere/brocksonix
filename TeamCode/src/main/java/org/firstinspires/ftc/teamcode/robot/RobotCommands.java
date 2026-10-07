package org.firstinspires.ftc.teamcode.robot;

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
    // TODO(12): Time a normal 1-ball shot, then add margin. Long enough that a good shot never times out, short enough that a misfeed doesn't eat the auto.
    public static double SHOOT_ONE_TIMEOUT_MS = 1500;
    // TODO(12): Time a full burst the same way.
    public static double SHOOT_ALL_TIMEOUT_MS = 3000;

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

    // ---- Intake and rail ----

    public Command flowerDown() {
        return Commands.instant(() -> robot.flowerIntake.setDown(true)).requiring(robot.flowerIntake);
    }

    public Command flowerUp() {
        return Commands.instant(() -> robot.flowerIntake.setDown(false)).requiring(robot.flowerIntake);
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
