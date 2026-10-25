package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
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
import org.firstinspires.ftc.teamcode.robot.subsystems.CoaxHood;
import org.firstinspires.ftc.teamcode.robot.subsystems.Door;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.robot.subsystems.FlowerIntake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Storage;
import org.firstinspires.ftc.teamcode.robot.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;

import java.util.EnumSet;
import java.util.Set;

/**
 * The whole robot. OpModes set what the drivers want (the public fields and
 * methods), then call update() once per loop, which decides what every
 * mechanism does and writes it to the hardware.
 *
 * Test OpModes skip this class and build only the subsystems they need.
 *
 * Two hoods are supported, and HOOD picks which one is used, live in Panels:
 * STATIC (fixed launch angle, the SHOT_RPM table) or COAX (the coax hood:
 * COAX_RPM for the wheel, and HoodSolver picks the angle from the wheel's real
 * speed). The coax hood servo is optional in the robot configuration. With it on
 * the robot and STATIC picked, the hood holds the static hood's angle, so the
 * static tables still fit and the two can be swapped at any time.
 */
@Configurable
public class Robot {
    // TODO(16): Switch to COAX once the coax hood is on the robot and its servo is in the configuration.
    /** Which hood shoots. Takes effect between bursts, never in the middle of one. */
    public static HoodType HOOD = HoodType.STATIC;
    // TODO(16): Main TeleOp, hold RB with 4 balls: raise for faster bursts while the hood makes up for the dip; lower if balls fired during recovery miss.
    /**
     * Coax hood only: the wheel must be within this many RPM of its target to fire. Wider than
     * Shooter.RPM_TOLERANCE on purpose: the hood angle is solved for the real RPM.
     */
    public static double COAX_RPM_TOLERANCE = 300;

    // OpModes may call these directly: drive, the flower intake, the aim trims (turret, shooter and coax hood),
    // and setPose/relocalize. update() decides everything else (the intake and transfer, the door, and the shooter,
    // turret and coax hood targets) from scratch every loop, so a direct call there is overwritten on the next update().
    // To change those, set shootHeld, passHeld, unjamHeld, singleBallBursts or stopAfterOneShot instead.
    public final Drive drive;
    public final Storage storage;
    public final Intake intake;
    public final Transfer transfer;
    public final Door door;
    public final FlowerIntake flowerIntake;
    public final Shooter shooter;
    public final Turret turret;
    /** The coax hood, or null when the robot configuration has no hood servo (HardwareNames.HOOD_SERVO). */
    public final CoaxHood coaxHood;
    /** Battery voltage, shared by the shooter and turret. Other code (like AgateFlow) can read it too. */
    public final VoltageCache voltage;

    public boolean shootHeld = false;
    /** Lob balls to the pass target instead of shooting at the HIVE. Ignored while FireControl.PASS_ENABLED is false. */
    public boolean passHeld = false;
    /** Runs the rail and intake backwards to clear a jam or spit balls out. */
    public boolean unjamHeld = false;
    /** Auto only: lets a burst start with 1 ball stored. TeleOp leaves this off, so a burst needs 2. */
    public boolean singleBallBursts = false;
    /** Auto only: the burst ends by itself after the first shot is counted (RobotCommands.shoot()). */
    public boolean stopAfterOneShot = false;

    private final BulkReads bulkReads;
    private final ShotSolver solver = new ShotSolver();
    private final FireControl fireControl = new FireControl();

    private ShotSolution aim;
    /**
     * Feed power, flywheel RPM and turret angle from the last good aim. aim can go null
     * in the middle of a burst (NaN pose). The rail pauses then, but a ball already past
     * the door still needs a wheel and turret that stay where they were.
     */
    private double lastFeedPower = 0;
    private double lastRpm = 0;
    private double lastTurretDeg = 0;
    /** The pose or velocity had a NaN this loop (odometry glitch). Nothing is aimed or saved while true. */
    private boolean poseIsNaN = false;
    /** The shot is ready this loop: aimed, and the wheel at speed. Starts a burst. */
    private boolean ready = false;
    /** The coax hood shoots this loop (HOOD is COAX and the hood servo is configured). */
    private boolean coax = false;
    /** Coax hood: the wheel is near its target but no hood angle scores from here (too close or too far for the hood). */
    private boolean hoodOutOfReach = false;
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
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        door = new Door(hardwareMap);
        flowerIntake = new FlowerIntake(hardwareMap);
        shooter = new Shooter(hardwareMap, voltage);
        turret = new Turret(hardwareMap, voltage);
        // Optional: a robot with the static hood has no hood servo in its configuration.
        coaxHood = CoaxHood.isConfigured(hardwareMap) ? new CoaxHood(hardwareMap) : null;
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
        if (coaxHood != null) coaxHood.resetTrim();
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

    /** True when the coax hood shoots: HOOD is COAX and its servo is configured. Changes only between bursts. */
    public boolean usingCoaxHood() {
        return coax;
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

        // A Panels switch takes effect between bursts only: mid-burst it would change the RPM
        // and the hood under a ball that is already being fed.
        if (!isFiring()) {
            boolean nowCoax = HOOD == HoodType.COAX && coaxHood != null;
            if (nowCoax != coax) {
                // Each trim was set for the other hood (in COAX, driver 2 trims the hood, not the RPM).
                shooter.resetTrim();
                if (coaxHood != null) coaxHood.resetTrim();
            }
            coax = nowCoax;
        }

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
        // No aim in the middle of a burst (NaN pose) keeps the last good aim. The rail pauses then
        // (not aimed), but cutting the wheel would send a ball already past the door into a slowing wheel.
        turret.setTargetDeg(lastTurretDeg);
        // Spin up whenever a ball is stored, so the wheel is already at speed when shoot is pressed.
        // It stays up for the whole burst too, so the wheel isn't cut while the last ball is still going through.
        shooter.setTargetRpm((isFiring() || !storage.isEmpty()) ? lastRpm : 0);
        if (coaxHood != null) {
            aimHood();
            // Written here, not at the end with the others, so atTarget() below already counts
            // this loop's move. The ball takes longer than a loop to reach the hood anyway.
            coaxHood.update();
        }

        boolean aimed = aimed();
        ready = aimed && wheelReady();
        hoodOutOfReach = coax && aim != null && aim.valid && !aim.hoodCanScore() && wheelNearTarget();
        boolean enoughBalls = singleBallBursts ? !storage.isEmpty() : storage.hasAtLeastTwo();
        fireControl.update(wanted, enoughBalls, stopAfterOneShot,
                ready, aimed, shooter.rpm(), shooter.effectiveTargetRpm(), door.isFullyOpen());
        shooter.setBoost(isFiring());

        // Open for the whole burst: the rail feeds without stopping, and the door never closes on a ball (see FireControl).
        if (fireControl.doorOpen()) door.open();
        else door.close();

        // Intake and transfer priority: unjam always wins, so a jam can be cleared at any time. Then feed.
        // Then stopped: while a burst waits, pauses or closes, the door is open, so the rail has to hold the balls back.
        // Collect is the default.
        if (unjamHeld) {
            intake.reverse();
            transfer.reverse();
        } else if (fireControl.feeding()) {
            intake.feed(lastFeedPower);
            transfer.feed(lastFeedPower);
        } else if (isFiring() || !door.isFullyClosed()) {
            // After a burst, stay stopped until the door is closed, or the next ball reaches it half open.
            intake.stop();
            transfer.stop();
        } else {
            intake.collect(storage.isFull());
            transfer.collect(storage.slot1Occupied());
        }

        door.update();
        flowerIntake.update();
        shooter.update();
        turret.update();

        // A guessed pose must not be saved, or a quick restart would restore it as trusted.
        // A NaN pose must not be saved either, or TeleOp would restore it as trusted.
        if (poseTrusted && !poseIsNaN) RobotState.savePose(pose);
    }

    /**
     * Where the coax hood goes. With STATIC picked it holds the static hood's angle. Otherwise
     * it follows the angle solved for the wheel's real RPM, for shots and passes alike. No angle
     * at this RPM (the wheel is still spinning up) or no aim in the middle of a burst (NaN pose):
     * it stays where it is, like the turret.
     */
    private void aimHood() {
        if (!coax) coaxHood.setTargetDeg(CoaxHood.STATIC_LAUNCH_DEG);
        else if (aim != null && aim.hoodCanScore()) coaxHood.setTargetDeg(aim.launchDeg);
        else if (aim == null && !isFiring()) coaxHood.setTargetDeg(CoaxHood.IDLE_LAUNCH_DEG);
    }

    /**
     * The aim is good: an alliance and a trusted pose, a target in range, and the turret on it.
     * Static hood: a coax hood on the robot must have reached the static angle. Coax hood, shots
     * and passes alike: the hood must have an angle that scores at the wheel's real RPM, and be
     * there. The wheel's speed is left out on purpose: mid-burst it sags with every ball, and
     * FireControl checks it against BURST_RPM_FLOOR instead.
     */
    private boolean aimed() {
        if (aim == null || !aim.valid || !canFire() || !turret.onTarget()) return false;
        if (!coax) return coaxHood == null || coaxHood.atTarget();
        return aim.hoodCanScore() && coaxHood.atTarget();
    }

    /**
     * Static hood: the wheel must be at speed (Shooter.RPM_TOLERANCE). Coax hood: it only needs
     * to be within COAX_RPM_TOLERANCE, since the hood angle is solved for its real RPM. That is
     * what lets a coax burst start sooner. GNCE Ditto went further and ignored the RPM entirely
     * once an angle existed (readyToShoot in
     * https://github.com/GNCE/DECODEv2-Ditto/blob/c78b5268930d55c6d5a59c23922d9a8baf9f96e6/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/subsystems/Shooter.java).
     */
    private boolean wheelReady() {
        return coax ? wheelNearTarget() : shooter.atSpeed();
    }

    private boolean wheelNearTarget() {
        double target = shooter.effectiveTargetRpm();
        return target > 0 && Math.abs(shooter.rpm() - target) < COAX_RPM_TOLERANCE;
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
        // The coax hood solves with the averaged RPM: steadier, so the hood servo doesn't shake.
        if (wanted == FireControl.Mode.PASS) {
            Pose passTarget = Field.passTarget(RobotState.alliance);
            return coax ? solver.solveCoaxPass(pose, passTarget, shooter.averageRpm())
                    : solver.solvePass(pose, passTarget);
        }
        RobotState.targetCell = Field.cellForSide(pose.y(), RobotState.targetCell);
        Pose cell = Field.cellTarget(RobotState.alliance, RobotState.targetCell);
        return coax ? solver.solveCoaxShot(pose, drive.velocity(), cell, shooter.averageRpm())
                : solver.solveShot(pose, drive.velocity(), cell);
    }

    /** The problems active as of the last update(). It is a new set each call, so the caller may keep it. */
    public Set<Warning> warnings() {
        Set<Warning> active = EnumSet.noneOf(Warning.class);
        if (RobotState.alliance == null) active.add(Warning.NO_ALLIANCE);
        else if (!poseTrusted) active.add(Warning.POSE_UNKNOWN);
        if (poseIsNaN) active.add(Warning.POSE_NAN);
        if ((aim != null && !aim.valid) || hoodOutOfReach) active.add(Warning.OUT_OF_RANGE);
        if (shooter.watchdogTripped()) active.add(Warning.SHOOTER_OFF);
        if (turret.watchdogTripped()) active.add(Warning.TURRET_OFF);
        if (ShotTables.orderProblem() != null) active.add(Warning.SHOT_TABLE_ORDER);
        if (HOOD == HoodType.COAX && coaxHood == null) active.add(Warning.HOOD_MISSING);
        if (coaxHood != null && coaxHood.servoAtEnd()) active.add(Warning.HOOD_AT_END);
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
        if (ShotSolver.LEAD || ShotSolver.PREDICT) solver.addTelemetry(telemetry, drive.velocity());
        if (!coax) {
            telemetry.addData("Hood", coaxHood == null ? "static" : "static (coax hood holding the static angle)");
        } else if (aim != null && aim.hoodCanScore()) {
            telemetry.addData("Hood", "coax, %.1f deg at %.0f rpm", aim.launchDeg, shooter.averageRpm());
        } else {
            telemetry.addData("Hood", "coax, no angle scores at %.0f rpm", shooter.averageRpm());
        }
        drive.addTelemetry(telemetry);
        storage.addTelemetry(telemetry);
        intake.addTelemetry(telemetry);
        transfer.addTelemetry(telemetry);
        door.addTelemetry(telemetry);
        flowerIntake.addTelemetry(telemetry);
        shooter.addTelemetry(telemetry);
        turret.addTelemetry(telemetry);
        if (coaxHood != null) coaxHood.addTelemetry(telemetry);
    }
}
