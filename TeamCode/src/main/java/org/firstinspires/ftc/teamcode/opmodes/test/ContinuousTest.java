package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.Corner;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.hardware.BulkReads;
import org.firstinspires.ftc.teamcode.robot.hardware.VoltageCache;
import org.firstinspires.ftc.teamcode.robot.subsystems.Door;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.robot.shot.TurnPredictor;
import org.firstinspires.ftc.teamcode.util.Angles;
import org.firstinspires.ftc.teamcode.util.PIDF;

/**
 * Continuous shooting with the real Intake, Transfer, Shooter, and Door
 * subsystems. Cross turns the shooter on and off; while on it runs at full power
 * (open loop, no RPM target). Hold RB to fire: the door opens and both rail motors
 * feed once it is fully open. Letting go of RB, or the shooter off, closes the door
 * and stops the rail, so nothing fires. LT reverses the rail motors either way.
 * No break beams are needed.
 *
 * Turret tracking: the Pinpoint tracks the robot like in Shooter Test. At Start the
 * pose is set to the NEAR_RIGHT corner (red's GARDEN corner, intake flat on the
 * alliance wall, side flat on the audience wall), so start the robot there; Options
 * sets it there again. Square turns tracking on/off: on, the turret (the real Turret
 * subsystem, Axon CR servo + encoder) keeps pointing at the cell (FAR_CELL picks
 * which) as the robot drives and turns; D-pad left/right switches it to the cell on the
 * red driver's left (FAR) or right (AUDIENCE). Off, the turret servo stops.
 * Tracking is predictive (shot/TurnPredictor, Triangle turns it off and on): from the
 * turn power sent to the wheels it aims from the heading the robot is about to have and
 * feeds the predicted turn rate to the turret as feedforward (Turret.KV), so the turret
 * turns against the robot the moment the turn stick moves. Compare "Yaw rate predicted /
 * measured" to tune TurnPredictor (Panels page "TurnPredictor"). Tune the turret
 * in Turret Test first (step 5): encoder volts, zero, directions, gains.
 *
 * Before the first run, check each shooter motor at low power in Shooter Test.
 * Sticks drive with Drive Test's stick code, field-centric for red (stick up = away from
 * the red wall, like Main TeleOp); Circle switches to robot-centric and back. With no
 * heading (NaN pose) it drives robot-centric. RT (hold) is slow mode.
 *
 * Pinpoint check: the "Pinpoint" line shows the device status, its loop time and firmware.
 * If the heading freezes, read it before power cycling: READY with a loop time means the
 * Pinpoint runs but its IMU stopped; FAULT_BAD_READ or NOT_READY means the whole device
 * stopped answering (its cable or its power). Share, with the robot still, resets the
 * Pinpoint and its IMU and puts the pose back at the NEAR_RIGHT corner half a second later.
 *
 * Panels field (practice only): the robot outline, the goal, a yellow line to the goal and
 * an orange line where the turret points. On target, the two lines lie on top of each other.
 */
@Configurable
@TeleOp(name = "Continuous Test", group = "Test")
public class ContinuousTest extends LinearOpMode {
    /** Which cell the turret tracks: true = FAR cell, false = AUDIENCE cell. Red field coordinates. */
    public static boolean FAR_CELL = false;
    /** Smooths "Yaw rate measured" (only telemetry): 0 = none, closer to 1 = smoother. */
    public static double YAW_RATE_FILTER = 0.5;
    /** How long Share's Pinpoint reset waits before setting the pose (the IMU takes about 0.25 s). */
    public static double PINPOINT_RESET_WAIT_SEC = 0.5;

    @Override
    public void runOpMode() {
        Telemetry screen = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        BulkReads bulkReads = new BulkReads(hardwareMap);
        VoltageCache voltage = new VoltageCache(hardwareMap);
        // Moves nothing: Mecanum only sets directions and float mode.
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);

        // Use the match subsystems so directions, control, and tuning stay consistent.
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap, voltage);
        Door door = new Door(hardwareMap);
        // Moves nothing in init: the Pinpoint only reads, the turret only reads its encoder.
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        Turret turret = new Turret(hardwareMap, voltage);
        // The same driver object the localizer uses, only for its status and Share's reset.
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
        int pinpointFirmware = pinpoint.getDeviceVersion();

        screen.addLine("Ready. Start the robot in the NEAR_RIGHT corner. Cross turns the shooter on, then hold RB to fire. Square: turret tracking.");
        screen.update();

        waitForStart();
        gamepad1.resetEdgeDetection();
        boolean shooterOn = false;
        boolean tracking = false;
        boolean fieldCentric = true;
        Pose startPose = Field.cornerPose(Alliance.RED, Corner.NEAR_RIGHT);
        localizer.setPose(startPose);
        TurnPredictor turnPredictor = new TurnPredictor();
        ElapsedTime loopTimer = new ElapsedTime();
        double lastHeading = 0;
        boolean hadHeading = false;
        double measuredYawRate = 0;
        ElapsedTime pinpointResetTimer = new ElapsedTime();
        boolean pinpointResetting = false;

        while (opModeIsActive()) {
            bulkReads.clear();
            shooter.readSensors();
            turret.readSensors();
            if (gamepad1.optionsWasPressed()) localizer.setPose(startPose);
            if (gamepad1.shareWasPressed()) {
                // Robot must be still: the Pinpoint zeroes its gyro while it calibrates.
                pinpoint.resetPosAndIMU();
                pinpointResetTimer.reset();
                pinpointResetting = true;
            }
            if (pinpointResetting && pinpointResetTimer.seconds() >= PINPOINT_RESET_WAIT_SEC) {
                localizer.setPose(startPose);
                pinpointResetting = false;
            }
            localizer.update();
            Pose pose = localizer.pose();
            boolean poseOk = !Double.isNaN(pose.x()) && !Double.isNaN(pose.y()) && !Double.isNaN(pose.heading());

            if (gamepad1.crossWasPressed()) shooterOn = !shooterOn;
            if (gamepad1.squareWasPressed()) tracking = !tracking;
            // Temporary: D-pad left/right picks the cell on the red driver's left (FAR) or right (AUDIENCE).
            if (gamepad1.dpadLeftWasPressed()) FAR_CELL = Field.cellFromDriver(Alliance.RED, true) == Cell.FAR;
            if (gamepad1.dpadRightWasPressed()) FAR_CELL = Field.cellFromDriver(Alliance.RED, false) == Cell.FAR;
            if (gamepad1.triangleWasPressed()) TurnPredictor.ENABLED = !TurnPredictor.ENABLED;

            // Drive first, so the turn predictor knows the turn power going out this loop.
            if (gamepad1.circleWasPressed()) fieldCentric = !fieldCentric;
            DrivePowers powers = DriveTest.stickPowers(gamepad1);
            // A NaN heading would make every power NaN, so field-centric waits for a real one.
            if (fieldCentric && poseOk) powers = Drive.driverToField(powers, pose.heading(), Alliance.RED);
            drivetrain.drive(powers, true);

            double dt = loopTimer.seconds();
            loopTimer.reset();
            turnPredictor.update(powers, dt);
            if (poseOk) {
                // The real turn rate, from the Pinpoint, to check the predictor against.
                double turned = Angles.wrapDegrees(Math.toDegrees(pose.heading() - lastHeading));
                if (hadHeading && dt > 0) measuredYawRate = PIDF.smooth(measuredYawRate, turned / dt, YAW_RATE_FILTER);
                lastHeading = pose.heading();
                hadHeading = true;
            } else {
                hadHeading = false;
            }

            Pose cell = Field.cellTarget(Alliance.RED, FAR_CELL ? Cell.FAR : Cell.AUDIENCE);
            if (tracking && poseOk) {
                // ShotSolver's static aim (field angle to the cell, minus the robot's heading), but
                // from the heading the robot will have once the turret gets there. The turret's
                // target then turns against the robot's predicted turn rate, which goes straight
                // in as feedforward, so the turret starts turning the moment the stick moves.
                double fieldAngle = Math.atan2(cell.y() - pose.y(), cell.x() - pose.x());
                double headingAhead = Math.toDegrees(pose.heading()) + turnPredictor.headingAheadDeg();
                turret.setTestPower(null);
                turret.setTargetDeg(Angles.wrapDegrees(Math.toDegrees(fieldAngle) - headingAhead));
                if (TurnPredictor.ENABLED) turret.setTargetRateDegPerSec(-turnPredictor.feedforwardRateDegPerSec());
            } else {
                // Tracking off, or no pose to aim from: the servo stops where it is.
                turret.setTestPower(0.0);
            }
            // Full power on both motors while on. Open loop skips the watchdog, so watch the wheel.
            shooter.setTestPowers(shooterOn ? 1 : 0, shooterOn ? 1 : 0);

            // Fires only while RB is held with the shooter on.
            boolean fire = shooterOn && gamepad1.right_bumper;
            if (fire) door.open();
            else door.close();
            boolean reversing = gamepad1.left_trigger > MainTeleOp.TRIGGER_THRESHOLD;
            String railState;
            double intakePower;
            double transferPower;
            if (reversing) {
                intake.reverse();
                transfer.reverse();
                railState = "reversing (LT)";
                intakePower = Math.max(Intake.REVERSE_POWER, -Intake.MAX_POWER);
                transferPower = Transfer.REVERSE_POWER;
            } else if (!fire) {
                intake.stop();
                transfer.stop();
                railState = shooterOn ? "stopped (hold RB to fire)" : "stopped (shooter off)";
                intakePower = 0;
                transferPower = 0;
            } else if (door.isFullyOpen()) {
                intake.collect(false);
                transfer.collect(false);
                railState = "running";
                intakePower = Math.min(Intake.COLLECT_POWER, Intake.MAX_POWER);
                transferPower = Transfer.COLLECT_POWER;
            } else {
                // Let the door finish opening before pushing balls into it.
                intake.stop();
                transfer.stop();
                railState = "waiting for door";
                intakePower = 0;
                transferPower = 0;
            }

            shooter.update();
            door.update();
            turret.update();

            // Panels field: robot, goal, the line to it (yellow) and where the turret points (orange).
            ShooterTest.drawField(poseOk ? pose : null, cell,
                    pose.heading() + Math.toRadians(turret.angleDeg()));

            screen.addData("Intake", "%s at %.2f power",
                    railState, intakePower);
            screen.addData("Transfer", "%s at %.2f power",
                    railState, transferPower);
            screen.addData("Shooter (Cross)", shooterOn ? "on, full power" : "off");
            screen.addData("Drive (Circle)", !fieldCentric ? "robot-centric"
                    : poseOk ? "field-centric" : "field-centric, no heading: robot-centric for now");
            screen.addData("Slow mode (RT)", DriveTest.slowHeld(gamepad1) ? "on" : "off");
            screen.addData("Turn prediction (Triangle)", TurnPredictor.ENABLED
                    ? String.format("on, lead %.1f deg, feedforward %.0f deg/s",
                            turnPredictor.headingAheadDeg(), turnPredictor.feedforwardRateDegPerSec()) : "off");
            screen.addData("Yaw rate predicted / measured", "%.0f / %.0f deg/s",
                    turnPredictor.rateDegPerSec(), measuredYawRate);
            screen.addData("Goal (D-pad left/right)", FAR_CELL ? "FAR cell (left)" : "AUDIENCE cell (right)");
            screen.addData("Turret tracking (Square)", tracking
                    ? (turret.onTarget() ? "on, on target" : "on, turning") : "off, servo stopped");
            if (poseOk) {
                screen.addData("Pose (Options: back to the GARDEN corner)", "x %.1f  y %.1f  heading %.0f deg",
                        pose.x(), pose.y(), Math.toDegrees(pose.heading()));
                screen.addData("Distance to " + (FAR_CELL ? "FAR" : "AUDIENCE") + " cell", "%.1f in.",
                        Math.hypot(cell.x() - pose.x(), cell.y() - pose.y()));
            } else {
                screen.addLine("!! Pose is NaN: the Pinpoint is not reading, so the turret can't track. Check its cable and pods.");
            }
            GoBildaPinpointDriver.DeviceStatus pinpointStatus = pinpoint.getDeviceStatus();
            screen.addData("Pinpoint (Share: reset, robot still)", "%s, loop %d us, firmware v%d%s",
                    pinpointStatus, pinpoint.getLoopTime(), pinpointFirmware,
                    pinpointResetting ? ", resetting" : "");
            if (pinpointStatus != GoBildaPinpointDriver.DeviceStatus.READY && !pinpointResetting) {
                screen.addLine("!! Pinpoint is not READY: the pose may be stale. Note this status before power cycling.");
            }
            shooter.addTelemetry(screen);
            door.addTelemetry(screen);
            turret.addTelemetry(screen);
            screen.addLine("Sticks: drive. Circle: field/robot-centric. RT: slow. Cross: shooter on/off. RB (hold, shooter on): fire. LT: reverse intake and transfer. Square: turret tracking. Triangle: turn prediction. D-pad left/right: goal. Options: reset pose. Share: reset Pinpoint.");
            screen.update();
        }

        drivetrain.stop();
        intake.stop();
        transfer.stop();
        shooter.setTestPowers(0, 0);
        shooter.update();
        door.close();
        door.update();
        turret.setTestPower(0.0);
        turret.update();
    }
}
