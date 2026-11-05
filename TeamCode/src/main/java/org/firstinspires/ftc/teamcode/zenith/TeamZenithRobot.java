package org.firstinspires.ftc.teamcode.zenith;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotState;
import org.firstinspires.ftc.teamcode.robot.commands.RobotCommands;
import org.horizon36596.zenith.AutoFile;
import org.horizon36596.zenith.ivy.AutoFromFile;
import org.horizon36596.zenith.ivy.NamedCommands;
import org.horizon36596.zenith.ivy.RobotTimeout;
import org.horizon36596.zenith.ivy.ZenithRobot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Our robot as Zenith's runtime sees it (zenith.json, deploy.robotClass). An auto file
 * drives paths through Pedro and calls our RobotCommands by name; Robot does the rest,
 * exactly as in AutoTemplate.
 *
 * - Loop: Zenith runs periodic(), Scheduler.execute(), then afterLoop(). periodic() does
 *   nothing and afterLoop() calls robot.update(), so the order is ours (commands, then
 *   robot.update()) and the follower is updated once per loop, inside Robot.update().
 * - Init writes no motor or servo: it builds Robot (same as AutoTemplate) and sets the pose.
 * - Alliance: Zenith flips a pose for blue through mirror(), once (see ZenithAuto.alliance).
 *   So the start pose goes to Robot.setPose, not Robot.setStartPose, which would flip it again.
 *   It is trusted, so Robot.update() saves it to RobotState every loop for Main TeleOp.
 * - Command names in a file are the RobotCommands method names (registerCommands).
 * - Every path ends: Zenith's FollowPath holds the robot when it is cut off, and a path step
 *   without its own timeoutS gets PATH_TIMEOUT_SECONDS (step()).
 * - A path's speedFraction becomes a Foresight maxPathSpeed limit on that path (atSpeed).
 *   robot.stop() at the end puts it back, as in AutoTemplate.
 */
@Configurable
public class TeamZenithRobot implements ZenithRobot {
    // TODO(9): Time the longest path in your autos at full speed, then add margin. Too short cuts real paths; too long lets a stuck robot push.
    /**
     * A path step with no timeoutS of its own gives up after this many seconds at full speed,
     * longer when slower (divided by its speedFraction), and holds where it is. Read when the
     * routine is built, at init.
     */
    public static double PATH_TIMEOUT_SECONDS = 6;

    private AutoFromFile opMode;
    private Robot robot;
    private RobotCommands commands;
    /** The alliance the routine is built for. Null while none is picked: ZenithAuto won't start then. */
    private Alliance builtFor;
    /** The start pose, already on our field and on this alliance's side. */
    private Pose startPose;
    /** Every step in the file by its id, so step() can find each path step's timeout and speed. */
    private final Map<String, AutoFile.Step> steps = new HashMap<>();
    private ElapsedTime sinceTelemetry;

    /** zenith deploy's OpModes build it with no arguments. */
    public TeamZenithRobot() {}

    @Override
    public void init(AutoFromFile opMode) {
        if (!(opMode instanceof ZenithAuto)) {
            throw new IllegalStateException(opMode.getClass().getSimpleName() + " must extend ZenithAuto"
                    + " (zenith.json deploy.opModeBase): it moves Zenith's poses onto our field and picks the alliance.");
        }
        this.opMode = opMode;
        robot = ((ZenithAuto) opMode).sharedRobot();
        commands = new RobotCommands(robot);
        builtFor = RobotState.alliance;
        sinceTelemetry = new ElapsedTime();
        steps.clear();
        addSteps(opMode.spec().steps);
    }

    private void addSteps(List<AutoFile.Step> list) {
        for (AutoFile.Step step : list) {
            steps.put(step.id, step);
            if (step instanceof AutoFile.SequenceStep) addSteps(((AutoFile.SequenceStep) step).steps);
            if (step instanceof AutoFile.ParallelStep) addSteps(((AutoFile.ParallelStep) step).steps);
            if (step instanceof AutoFile.BranchStep) {
                addSteps(((AutoFile.BranchStep) step).then);
                addSteps(((AutoFile.BranchStep) step).otherwise);
            }
        }
    }

    /**
     * The names an auto file can use. Each is the RobotCommands method it calls, so a file
     * reads like AutoTemplate's routine. A file can only pass numbers, so the methods that
     * take a pose, a path or another command (driveTo, follow, sweepAndShoot, ...) are not
     * here: driving is the file's own path steps.
     */
    @Override
    public void registerCommands(AutoFromFile opMode) {
        NamedCommands.register("shoot", (args, ctx) -> commands.shoot());
        NamedCommands.register("shootAll", (args, ctx) -> commands.shootAll());
        NamedCommands.register("shootAllWithRetry", (args, ctx) -> commands.shootAllWithRetry());
        // Never ends by itself: put it in a parallel step whose deadline is a path.
        NamedCommands.register("shootContinuous", (args, ctx) -> commands.shootContinuous());
        NamedCommands.register("flowerDown", (args, ctx) -> commands.flowerDown());
        NamedCommands.register("flowerUp", (args, ctx) -> commands.flowerUp());
        NamedCommands.register("waitUntilFull", (args, ctx) ->
                commands.waitUntilFull(NamedCommands.argDouble(args, "waitUntilFull", "timeoutMs")));
        NamedCommands.register("unjam", (args, ctx) ->
                commands.unjam(NamedCommands.argDouble(args, "unjam", "ms")));

        // For branch, "wait until" and a path's endCondition.
        NamedCommands.registerCondition("storageEmpty", () -> robot.storage.isEmpty());
        NamedCommands.registerCondition("storageFull", () -> robot.storage.isFull());
        NamedCommands.registerCondition("canFire", () -> robot.canFire());
    }

    @Override
    public Follower follower() {
        return robot.drive.follower();
    }

    /** What every path step requires: the same object RobotCommands' drives require, so two drives never run at once. */
    @Override
    public Object drive() {
        return robot.drive;
    }

    @Override
    public String runningAlliance() {
        // No alliance yet: build the file as written. ZenithAuto builds it again once one is picked.
        return builtFor != null ? builtFor.name() : opMode.spec().alliance;
    }

    /** Blue's half is red's spun 180° about the center, so Alliance.fromRed flips either way. */
    @Override
    public Pose mirror(Pose pose) {
        return Alliance.BLUE.fromRed(pose);
    }

    @Override
    public void setStartPose(Pose pose) {
        startPose = pose;
        if (builtFor != null) robot.setPose(pose, true);
    }

    /** Set again at start, as AutoTemplate does, so anything the localizer drifted while waiting is dropped. */
    @Override
    public void onStart(AutoFromFile opMode) {
        robot.setPose(startPose, true);
    }

    /** Nothing: robot.update() in afterLoop updates the follower. Updating it here too would run it twice. */
    @Override
    public void periodic(AutoFromFile opMode) {}

    /** Runs right after Scheduler.execute(), so commands set what to do and robot.update() does it this same loop. */
    @Override
    public void afterLoop(AutoFromFile opMode) {
        robot.update();
        // MainTeleOp.FULL_TELEMETRY and TELEMETRY_INTERVAL_MS set the auto's telemetry too.
        if (MainTeleOp.FULL_TELEMETRY || sinceTelemetry.milliseconds() >= MainTeleOp.TELEMETRY_INTERVAL_MS) {
            sinceTelemetry.reset();
            robot.addWarnings(opMode.telemetry);
            if (MainTeleOp.FULL_TELEMETRY) robot.addTelemetry(opMode.telemetry);
            opMode.telemetry.update();
        }
    }

    /** Auto usually ends in the middle of a path. This puts back its speed limit, so it can't slow down the next OpMode. */
    @Override
    public void onStop(AutoFromFile opMode) {
        if (robot != null) robot.stop();
    }

    /**
     * Foresight's maxPathSpeed is a fraction of the robot's top speed, so the file's
     * speedFraction goes straight in, as a Modifier on this one path.
     */
    @Override
    public Path atSpeed(Path path, double speedFraction) {
        Follower follower = follower();
        if (speedFraction >= 1 || !(follower.algorithm() instanceof Foresight)) return path;
        Foresight foresight = (Foresight) follower.algorithm();
        return path.with(foresight.config.maxPathSpeed.at(speedFraction));
    }

    /**
     * Zenith calls this for every step. Path steps get two checks here, at init:
     * - Markers are refused. Zenith's marker runner calls Follower.curveCompletion(), which
     *   our Pedro (3.0.1) doesn't have, so the auto would stop with an error on the field.
     * - A path with no timeoutS of its own gets one, so a robot that is stuck gives up and holds.
     *   It wraps the whole step, so a timeout ends the path and its end condition together.
     */
    @Override
    public Command step(String id, Command body) {
        AutoFile.Step step = steps.get(id);
        if (!(step instanceof AutoFile.PathStep)) return body;
        AutoFile.PathStep path = (AutoFile.PathStep) step;
        if (!path.markers.isEmpty()) {
            throw new IllegalStateException("Step " + id + " has markers, which can't run on our Pedro version."
                    + " Use a parallel step instead: mode deadline, the path as its deadline, the command beside it.");
        }
        if (path.timeoutS != null) return body;
        double speed = path.speedFraction != null ? path.speedFraction : opMode.defaultSpeedFraction();
        return RobotTimeout.of(body, PATH_TIMEOUT_SECONDS / Math.max(speed, 0.1), opMode);
    }
}
