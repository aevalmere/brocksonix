package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.field.Corner;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.shot.ShotSolution;
import org.firstinspires.ftc.teamcode.shot.ShotSolver;
import org.firstinspires.ftc.teamcode.subsystems.Door;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.FlowerIntake;
import org.firstinspires.ftc.teamcode.subsystems.Rail;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Storage;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.BulkReads;
import org.firstinspires.ftc.teamcode.util.VoltageCache;

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

    private final BulkReads bulkReads;
    private final ShotSolver solver = new ShotSolver();
    private final FireControl fireControl = new FireControl();

    private ShotSolution aim;
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

    /** Aiming needs the alliance and a trusted pose. Without them, nothing fires. */
    public boolean canFire() {
        return RobotState.alliance != null && poseTrusted;
    }

    public void update() {
        bulkReads.clear();
        drive.update();
        storage.update();

        Pose pose = drive.pose();
        aim = solve(pose);

        if (aim != null) {
            turret.setTargetDeg(aim.turretDeg);
            shooter.setTargetRpm(isFiring() || !storage.isEmpty() ? aim.rpm : 0);
        } else {
            turret.setTargetDeg(0);
            shooter.setTargetRpm(0);
        }

        ready = aim != null && aim.valid && canFire() && shooter.atSpeed() && turret.onTarget();
        fireControl.update(shootHeld, passHeld, storage.hasAtLeastTwo(),
                ready, shooter.rpm(), shooter.effectiveTargetRpm(), door.isFullyOpen());
        shooter.setBoost(isFiring());

        if (fireControl.doorOpen()) door.open();
        else door.close();

        if (unjamHeld) rail.reverse();
        else if (fireControl.feeding()) rail.feed(aim.feedPower);
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
        if (poseTrusted) RobotState.savePose(pose);
    }

    private ShotSolution solve(Pose pose) {
        if (RobotState.alliance == null) return null;
        if (passHeld && FireControl.PASS_ENABLED && !shootHeld) {
            return solver.solvePass(pose, Field.passTarget(RobotState.alliance));
        }
        RobotState.targetCell = Field.cellForSide(pose.y(), RobotState.targetCell);
        return solver.solveShot(pose, drive.velocity(), Field.cellTarget(RobotState.alliance, RobotState.targetCell));
    }

    /** Problems the drivers must see. Shown even when the rest of telemetry is off. */
    public void addWarnings(Telemetry telemetry) {
        if (RobotState.alliance == null) telemetry.addLine("!! NO ALLIANCE: restart the OpMode and pick one in init");
        else if (!poseTrusted) telemetry.addLine("!! POSE UNKNOWN: relocalize in a corner before shooting");
        if (aim != null && !aim.valid) telemetry.addLine("!! Target out of range");
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
