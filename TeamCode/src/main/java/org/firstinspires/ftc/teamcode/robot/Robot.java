package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.field.Corner;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.hardware.BulkReads;
import org.firstinspires.ftc.teamcode.robot.hardware.VoltageCache;
import org.firstinspires.ftc.teamcode.robot.shot.ShotSolution;
import org.firstinspires.ftc.teamcode.robot.shot.ShotSolver;
import org.firstinspires.ftc.teamcode.robot.shot.ShotTables;
import org.firstinspires.ftc.teamcode.robot.subsystems.Door;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.robot.subsystems.FlowerIntake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Rail;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Storage;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;

import java.util.EnumSet;
import java.util.Set;

/**
 * The whole robot. OpModes set what the drivers want (the public fields and
 * methods), then call update() once per loop, which decides what every
 * mechanism does and writes it to the hardware.
 *
 * Test OpModes skip this class and build only the subsystems they need.
 */
public class Robot {
    // OpModes may call these directly: drive, the flower intake, the aim trims (turret and shooter),
    // and setPose/relocalize. update() decides everything else (rail, door, shooter and turret targets)
    // from scratch every loop, so a direct call there is overwritten on the next update().
    // To change those, set shootHeld, passHeld, unjamHeld or singleBallBursts instead.
    public final Drive drive;
    public final Storage storage;
    public final Rail rail;
    public final Door door;
    public final FlowerIntake flowerIntake;
    public final Shooter shooter;
    public final Turret turret;
    /** Battery voltage, shared by the shooter and turret. Other code (like AgateFlow) can read it too. */
    public final VoltageCache voltage;

    public boolean shootHeld = false;
    /** Lob balls to the pass target instead of shooting at the HIVE. Ignored while FireControl.PASS_ENABLED is false. */
    public boolean passHeld = false;
    /** Runs the rail and intake backwards to clear a jam or spit balls out. */
    public boolean unjamHeld = false;
    /** Auto only: lets a burst start with 1 ball stored. TeleOp leaves this off, so a burst needs 2. */
    public boolean singleBallBursts = false;

    private final BulkReads bulkReads;
    private final ShotSolver solver = new ShotSolver();
    private final FireControl fireControl = new FireControl();

    private ShotSolution aim;
    /**
     * Feed power, flywheel RPM and turret angle from the last good aim. aim can go null
     * in the middle of a burst (NaN pose). A ball that is already moving still needs a
     * power to be pushed with, and a wheel and turret that stay where they were.
     */
    private double lastFeedPower = 0;
    private double lastRpm = 0;
    private double lastTurretDeg = 0;
    /** The pose or velocity had a NaN this loop (odometry glitch). Nothing is aimed or saved while true. */
    private boolean poseIsNaN = false;
    private boolean ready = false;
    private boolean wasFull = false;
    /** False until the pose is known: a recent saved pose, or a relocalize. Nothing fires while false. */
    private boolean poseTrusted = false;

    public Robot(HardwareMap hardwareMap) {
        bulkReads = new BulkReads(hardwareMap);
        voltage = new VoltageCache(hardwareMap);

        Follower follower = Constants.create(hardwareMap);
        if (follower == null) {
            throw new IllegalStateException("Constants.create() returns null. Run the Pedro tuners and fill it in first.");
        }

        drive = new Drive(follower);
        storage = new Storage(hardwareMap);
        rail = new Rail(hardwareMap);
        door = new Door(hardwareMap);
        flowerIntake = new FlowerIntake(hardwareMap);
        shooter = new Shooter(hardwareMap, voltage);
        turret = new Turret(hardwareMap, voltage);
    }

    /** @param trusted true if this pose is known to be right (fresh from auto, or just relocalized) */
    public void setPose(Pose pose, boolean trusted) {
        drive.setPose(pose);
        poseTrusted = trusted;
    }

    /**
     * Where the robot sits when auto starts. The pose is written for red and flipped here for blue.
     * Auto calls this once at the start. Since the pose is trusted, it is saved every loop for TeleOp.
     */
    public void setStartPose(Pose redPose) {
        if (RobotState.alliance == null) return;
        setPose(RobotState.alliance.fromRed(redPose), true);
    }

    /** Snap the pose to a corner the robot is pushed into. Trims are reset, since they mostly cover pose drift. */
    public void relocalize(Corner corner) {
        if (RobotState.alliance == null) return;
        setPose(Field.cornerPose(RobotState.alliance, corner), true);
        turret.resetTrim();
        shooter.resetTrim();
    }

    public boolean isFiring() {
        return fireControl.isFiring();
    }

    /** Shots counted since the Robot was built. It never resets, so compare against an earlier reading. */
    public int shotsFired() {
        return fireControl.totalShots();
    }

    /** Aiming needs the alliance and a trusted pose. Without them, nothing fires. */
    public boolean canFire() {
        return RobotState.alliance != null && poseTrusted;
    }

    /**
     * Call once, right after the OpMode's loop ends. Pedro writes a path's speed limit into
     * the Foresight settings and only puts the old value back when the path is let go.
     * Stopping the follower lets it go, so the limit isn't left behind for later OpModes.
     */
    public void stop() {
        drive.stop();
    }

    public void update() {
        // Decide shoot or pass once, so aiming and FireControl can't disagree. Shoot wins if both are held.
        FireControl.Mode wanted = shootHeld ? FireControl.Mode.SHOOT
                : (passHeld && FireControl.PASS_ENABLED) ? FireControl.Mode.PASS
                : FireControl.Mode.NONE;

        bulkReads.clear();
        drive.update();
        storage.update();
        // Read at the top, write at the end (the shooter.update() and turret.update() calls below),
        // so every decision this loop uses this loop's readings, not last loop's.
        shooter.readSensors();
        turret.readSensors();

        Pose pose = drive.pose();
        poseIsNaN = isBad(pose) || isBad(drive.velocity());
        aim = solve(pose, wanted);

        if (aim != null) {
            lastFeedPower = aim.feedPower;
            lastRpm = aim.rpm;
            lastTurretDeg = aim.turretDeg;
        } else if (!isFiring()) {
            // No aim and no burst: the turret goes to 0 and the wheel stops.
            lastRpm = 0;
            lastTurretDeg = 0;
        }
        // No aim in the middle of a burst (NaN pose) keeps the last good aim. Cutting the wheel
        // there would send the ball being fed into a slowing wheel, and count the slowdown as a shot.
        turret.setTargetDeg(lastTurretDeg);
        // Spin up whenever a ball is stored, so the wheel is already at speed when shoot is pressed.
        // It stays up for the whole burst too, so the wheel isn't cut while the last ball is still going through.
        shooter.setTargetRpm((isFiring() || !storage.isEmpty()) ? lastRpm : 0);

        ready = aim != null && aim.valid && canFire() && shooter.atSpeed() && turret.onTarget();
        boolean enoughBalls = singleBallBursts ? !storage.isEmpty() : storage.hasAtLeastTwo();
        fireControl.update(wanted, enoughBalls,
                ready, shooter.rpm(), shooter.effectiveTargetRpm(), door.isFullyOpen());
        shooter.setBoost(isFiring());

        // Open for the whole burst, not per ball: it never closes on a ball (see FireControl),
        // and the burst doesn't wait on the door between balls.
        if (fireControl.doorOpen()) door.open();
        else door.close();

        // Rail priority: unjam always wins, so a jam can be cleared at any time. Then feed.
        // Then stopped: the door stays open between balls, so the rail has to hold the next ball back.
        // Collect is the default.
        if (unjamHeld) rail.reverse();
        else if (fireControl.feeding()) rail.feed(lastFeedPower);
        // After a burst, stay stopped until the door is closed, or the next ball reaches it half open.
        else if (isFiring() || !door.isFullyClosed()) rail.stop();
        else rail.collect(storage.slot1Occupied(), storage.isFull());

        if (storage.isFull() && !wasFull) flowerIntake.setDown(false);
        wasFull = storage.isFull();

        door.update();
        flowerIntake.update();
        shooter.update();
        turret.update();

        // A guessed pose must not be saved, or a quick restart would restore it as trusted.
        // A NaN pose must not be saved either, or TeleOp would restore it as trusted.
        if (poseTrusted && !poseIsNaN) RobotState.savePose(pose);
    }

    private static boolean isBad(Pose pose) {
        return Double.isNaN(pose.x()) || Double.isNaN(pose.y()) || Double.isNaN(pose.heading());
    }

    private static boolean isBad(Velocity velocity) {
        return Double.isNaN(velocity.vx) || Double.isNaN(velocity.vy) || Double.isNaN(velocity.omega);
    }

    private ShotSolution solve(Pose pose, FireControl.Mode wanted) {
        if (RobotState.alliance == null) return null;
        // A NaN would send the flywheel to the top RPM and the turret to a NaN angle, so don't aim at all.
        if (poseIsNaN) return null;
        if (wanted == FireControl.Mode.PASS) {
            return solver.solvePass(pose, Field.passTarget(RobotState.alliance));
        }
        RobotState.targetCell = Field.cellForSide(pose.y(), RobotState.targetCell);
        return solver.solveShot(pose, drive.velocity(), Field.cellTarget(RobotState.alliance, RobotState.targetCell));
    }

    /** The problems active as of the last update(). It is a new set each call, so the caller may keep it. */
    public Set<Warning> warnings() {
        Set<Warning> active = EnumSet.noneOf(Warning.class);
        if (RobotState.alliance == null) active.add(Warning.NO_ALLIANCE);
        else if (!poseTrusted) active.add(Warning.POSE_UNKNOWN);
        if (poseIsNaN) active.add(Warning.POSE_NAN);
        if (aim != null && !aim.valid) active.add(Warning.OUT_OF_RANGE);
        if (shooter.watchdogTripped()) active.add(Warning.SHOOTER_OFF);
        if (turret.watchdogTripped()) active.add(Warning.TURRET_OFF);
        if (ShotTables.orderProblem() != null) active.add(Warning.SHOT_TABLE_ORDER);
        return active;
    }

    /** Problems the drivers must see. Shown even when the rest of telemetry is off. */
    public void addWarnings(Telemetry telemetry) {
        for (Warning warning : warnings()) telemetry.addLine(warning.text);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Alliance / cell", "%s / %s", RobotState.alliance, RobotState.targetCell);
        telemetry.addData("Fire", "%s%s", fireControl.status(), ready ? " (ready)" : "");
        if (aim != null) {
            // The real distance first. RPM and range use the led distance, which only differs while LEAD is moving the aim point.
            if (Math.round(aim.distance) != Math.round(aim.targetDistance)) {
                telemetry.addData("Aim", "%.0f in (with lead %.0f in), %.0f rpm", aim.targetDistance, aim.distance, aim.rpm);
            } else {
                telemetry.addData("Aim", "%.0f in, %.0f rpm", aim.distance, aim.rpm);
            }
        }
        drive.addTelemetry(telemetry);
        storage.addTelemetry(telemetry);
        rail.addTelemetry(telemetry);
        door.addTelemetry(telemetry);
        flowerIntake.addTelemetry(telemetry);
        shooter.addTelemetry(telemetry);
        turret.addTelemetry(telemetry);
    }
}
