package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.util.DriveCurve;

/**
 * Field-centric driving on top of Pedro's Follower: stick up always drives
 * away from the driver, whichever way the robot faces. When the sticks are
 * let go the robot holds its position, so a defender can't push it off a shot.
 *
 * Driving direction is shaped as one vector (not x and y separately), so
 * diagonals stay the angle the stick points.
 */
@Configurable
public class Drive {
    // TODO(2): Raise if the robot creeps with the sticks let go; lower if small moves feel dead.
    public static double STICK_DEADBAND = 0.05;
    // TODO(2): Drive and adjust DRIVE_CUBIC and TURN_CUBIC to taste. Higher = gentler near center, same top speed.
    public static double DRIVE_CUBIC = 0.6;
    public static double TURN_CUBIC = 0.7;
    // TODO(2): Drive around and let go. If holding feels jumpy, set false.
    public static boolean HOLD_WHEN_STOPPED = true;
    // TODO(2): Drive around and let go. Raise if it rolls on before holding; lower if the hold feels jumpy.
    /** Holding starts once the robot has slowed below this (inches/second). */
    public static double HOLD_BELOW_SPEED = 2;

    private final Follower follower;

    public Drive(Follower follower) {
        this.follower = follower;
    }

    // TODO(2): Main TeleOp, red: stick up drives away from the driver, stick left to the driver's left. Check blue too.
    /**
     * Raw stick values from the driver's point of view, each -1 to 1.
     *
     * @param away     positive drives away from the driver
     * @param left     positive drives to the driver's left
     * @param turnLeft positive turns counter-clockwise
     */
    public void drive(double away, double left, double turnLeft, Alliance alliance) {
        boolean driverInput = Math.hypot(away, left) > STICK_DEADBAND || Math.abs(turnLeft) > STICK_DEADBAND;
        if (!driverInput && HOLD_WHEN_STOPPED) {
            ManualDrive.driveOrHold(follower, DrivePowers.zero(), STICK_DEADBAND, HOLD_BELOW_SPEED);
            return;
        }

        double speed = Math.min(Math.hypot(away, left), 1);
        double shapedSpeed = DriveCurve.shape(speed, STICK_DEADBAND, DRIVE_CUBIC);
        double stretch = speed > 0 ? shapedSpeed / speed : 0;

        DrivePowers fromDriver = new DrivePowers(
                away * stretch,
                left * stretch,
                DriveCurve.shape(turnLeft, STICK_DEADBAND, TURN_CUBIC));
        // Pedro rotates by -(heading + offset), so offset = -driverHeading first turns the driver's view into the field's.
        follower.manual(ManualDrive.fieldCentric(fromDriver, pose().heading(), -alliance.driverHeading()));
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
        // Re-hold at the new pose, or the follower drives back to the old hold target.
        if (follower.holding()) follower.hold(pose);
    }

    public void update() {
        follower.update();
    }

    /** Stops following or holding. Any Foresight setting the current path changed (like a speed limit) is put back. */
    public void stop() {
        follower.stop();
    }

    public Pose pose() {
        return follower.pose();
    }

    /** Field-frame velocity: inches/second and radians/second. */
    public Velocity velocity() {
        return follower.velocity();
    }

    public Follower follower() {
        return follower;
    }

    public void addTelemetry(Telemetry telemetry) {
        Pose pose = pose();
        telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Follower mode", follower.mode());
    }
}
