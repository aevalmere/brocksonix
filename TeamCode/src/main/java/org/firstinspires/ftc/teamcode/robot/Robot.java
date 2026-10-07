package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.field.Corner;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.shot.ShotSolution;
import org.firstinspires.ftc.teamcode.shot.ShotSolver;
import org.firstinspires.ftc.teamcode.shot.ShotTables;
import org.firstinspires.ftc.teamcode.subsystems.Door;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.FlowerIntake;
import org.firstinspires.ftc.teamcode.subsystems.Rail;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Storage;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.BulkReads;
import org.firstinspires.ftc.teamcode.util.VoltageCache;

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
    public final Drive drive;
    public final Storage storage;
    public final Rail rail;
    public final Door door;
    public final FlowerIntake flowerIntake;
    public final Shooter shooter;
    public final Turret turret;

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
     * Feed power from the last good aim. aim can go null in the middle of a feed
     * (NaN pose), and a ball that is already moving still needs a power to be pushed with.
     */
    private double lastFeedPower = 0;
    /** The pose or velocity had a NaN this loop (odometry glitch). Nothing is aimed or saved while true. */
    private boolean poseIsNaN = false;
    private boolean ready = false;
    private boolean wasFull = false;
    /** False until the pose is known: a recent saved pose, or a relocalize. Nothing fires while false. */
    private boolean poseTrusted = false;

    public Robot(HardwareMap hardwareMap) {
        bulkReads = new BulkReads(hardwareMap);
        VoltageCache voltage = new VoltageCache(hardwareMap);

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

    public void update() {
        bulkReads.clear();
        drive.update();
        storage.update();

        Pose pose = drive.pose();
        poseIsNaN = isBad(pose) || isBad(drive.velocity());
        aim = solve(pose);

        if (aim != null) {
            lastFeedPower = aim.feedPower;
            turret.setTargetDeg(aim.turretDeg);
            shooter.setTargetRpm(isFiring() || !storage.isEmpty() ? aim.rpm : 0);
        } else {
            turret.setTargetDeg(0);
            shooter.setTargetRpm(0);
        }

        ready = aim != null && aim.valid && canFire() && shooter.atSpeed() && turret.onTarget();
        boolean enoughBalls = singleBallBursts ? !storage.isEmpty() : storage.hasAtLeastTwo();
        fireControl.update(shootHeld, passHeld, enoughBalls,
                ready, shooter.rpm(), shooter.effectiveTargetRpm(), door.isFullyOpen());
        shooter.setBoost(isFiring());

        if (fireControl.doorOpen()) door.open();
        else door.close();

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

    private ShotSolution solve(Pose pose) {
        if (RobotState.alliance == null) return null;
        // A NaN would send the flywheel to the top RPM and the turret to a NaN angle, so don't aim at all.
        if (poseIsNaN) return null;
        if (passHeld && FireControl.PASS_ENABLED && !shootHeld) {
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
            telemetry.addData("Aim", "%.0f in, %.0f rpm", aim.distance, aim.rpm);
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
