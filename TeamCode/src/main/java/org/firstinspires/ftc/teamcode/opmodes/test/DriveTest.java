package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.hardware.BulkReads;
import org.firstinspires.ftc.teamcode.robot.hardware.VoltageCache;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.util.Angles;

/**
 * First drive, before any tuning. It builds only Pedro's Mecanum drivetrain and
 * Pinpoint localizer, from Constants.drivetrainConfig and localizerConfig: no
 * Follower, no Robot. Start on blocks or in open space.
 *
 * D-pad (hold): spins one wheel at WHEEL_TEST_POWER. Up = front left, right =
 * front right, down = back right, left = back left. Each must turn the way that
 * rolls the robot forward (intake first). If one turns backward, flip its
 * direction in Constants.drivetrainConfig. If a different wheel spins, its port
 * is wrong. The D-pad overrides the sticks.
 *
 * Sticks start robot-centric: left stick up drives intake first, left stick
 * left strafes left, right stick left turns left (counter-clockwise from
 * above). Right trigger (hold): slow mode, all three sticks times SLOW_SCALE.
 *
 * Cross: field-centric and back. In field-centric, stick up drives along
 * heading 0. Red's driver heading is 0, so press Options with the intake
 * pointed away from you, and stick up then drives away from you. While the
 * heading reads NaN, the sticks drive robot-centric.
 *
 * Options: pose to (0, 0, 0). Then push the robot 48 in. forward (x rises 48),
 * 48 in. left (y rises 48), and turn it left (heading goes positive). If x or y
 * runs the wrong way, flip that pod's direction in Constants.localizerConfig.
 */
@Configurable
@TeleOp(name = "Drive Test", group = "Test")
public class DriveTest extends LinearOpMode {
    /** Power for the one-wheel check (D-pad). Keep it low. */
    public static double WHEEL_TEST_POWER = 0.3;
    /** Right trigger held multiplies all three sticks by this. */
    public static double SLOW_SCALE = 0.3;

    // In D-pad order: up, right, down, left.
    private static final String[] WHEEL_NAMES = {"front left", "front right", "back right", "back left"};

    @Override
    public void runOpMode() {
        Telemetry screen = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        BulkReads bulkReads = new BulkReads(hardwareMap);
        VoltageCache voltage = new VoltageCache(hardwareMap);
        // Neither moves anything: Mecanum only sets directions and float mode, the Pinpoint recalibrates its IMU.
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        // hardwareMap hands back the same motor objects Mecanum got, so these carry the config's directions.
        DcMotorEx[] wheels = {
                hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.frontLeftName.get()),
                hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.frontRightName.get()),
                hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.backRightName.get()),
                hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.backLeftName.get()),
        };
        int spinningWheel = -1; // index into wheels, -1 = none
        boolean fieldCentric = false;

        screen.addLine("Ready. Robot on blocks or in open space.");
        screen.update();

        waitForStart();
        // "WasPressed" remembers presses made during init, which would fire on the first loop.
        gamepad1.resetEdgeDetection();
        localizer.setPose(new Pose(0, 0, 0));

        ElapsedTime loopTimer = new ElapsedTime();
        while (opModeIsActive()) {
            double loopMs = loopTimer.milliseconds();
            loopTimer.reset();
            bulkReads.clear();

            if (gamepad1.crossWasPressed()) fieldCentric = !fieldCentric;
            if (gamepad1.optionsWasPressed()) localizer.setPose(new Pose(0, 0, 0));
            localizer.update();
            Pose pose = localizer.pose();
            boolean noHeading = Double.isNaN(pose.heading());

            int wheel = -1;
            if (gamepad1.dpad_up) wheel = 0;
            else if (gamepad1.dpad_right) wheel = 1;
            else if (gamepad1.dpad_down) wheel = 2;
            else if (gamepad1.dpad_left) wheel = 3;
            // Mecanum skips a write when its cache already holds that power, so after a direct spin it
            // might never send the 0. On every D-pad change, stop all four through Mecanum (its cache
            // goes to 0) and directly (the wheels really stop).
            if (wheel != spinningWheel) {
                stopAll(drivetrain, wheels);
                spinningWheel = wheel;
            }

            boolean slow = slowHeld(gamepad1);
            if (wheel >= 0) {
                wheels[wheel].setPower(WHEEL_TEST_POWER);
            } else {
                DrivePowers powers = stickPowers(gamepad1);
                // A NaN heading would make every power NaN, so field-centric waits for a real one.
                if (fieldCentric && !noHeading) {
                    // Main TeleOp's rotation, for red.
                    powers = Drive.driverToField(powers, pose.heading(), Alliance.RED);
                }
                // true = driver input, so the wheels brake at 0 power (as in Main TeleOp).
                drivetrain.drive(powers, true);
            }

            screen.addData("Drive (Cross)", fieldCentric ? "field-centric" : "robot-centric");
            screen.addLine("D-pad: one wheel. Sticks: drive. Cross: field/robot. RT: slow. Options: pose to 0.");
            if (Double.isNaN(pose.x()) || Double.isNaN(pose.y()) || noHeading) {
                screen.addLine("!! Pose is NaN: the Pinpoint is not reading. Check its cable and pods.");
            }
            if (fieldCentric && noHeading) {
                screen.addLine("!! No heading, so the sticks drive robot-centric until it reads.");
            }
            if (spinningWheel >= 0) {
                screen.addData("D-pad wheel", "%s at %.2f power", WHEEL_NAMES[spinningWheel], WHEEL_TEST_POWER);
            } else {
                screen.addData("D-pad wheel", "none, sticks drive");
            }
            screen.addData("Slow mode (RT)", slow ? "on" : "off");
            screen.addData("x", "%.1f in.", pose.x());
            screen.addData("y", "%.1f in.", pose.y());
            screen.addData("Heading", "%.1f deg", Angles.wrapDegrees(Math.toDegrees(pose.heading())));
            screen.addData("Loop", "%.1f ms", loopMs);
            screen.addData("Battery", "%.2f V", voltage.get());
            screen.update();
        }
        stopAll(drivetrain, wheels);
    }

    /**
     * Robot-centric drive powers from the sticks: deadband, then SLOW_SCALE while
     * RT is held. Standard signs, as in Main TeleOp: a stick reads -1 pushed up
     * or left, and Pedro takes forward, left and counter-clockwise as positive.
     * Intake Test drives with this too, so a sign changed here changes both OpModes.
     */
    static DrivePowers stickPowers(Gamepad pad) {
        double forward = -pad.left_stick_y;
        double left = -pad.left_stick_x;
        double turnLeft = -pad.right_stick_x;
        if (Math.hypot(forward, left) <= Drive.STICK_DEADBAND) {
            forward = 0;
            left = 0;
        }
        if (Math.abs(turnLeft) <= Drive.STICK_DEADBAND) turnLeft = 0;
        double scale = slowHeld(pad) ? SLOW_SCALE : 1;
        return new DrivePowers(forward * scale, left * scale, turnLeft * scale);
    }

    /** Right trigger held: slow mode. */
    static boolean slowHeld(Gamepad pad) {
        return pad.right_trigger > MainTeleOp.TRIGGER_THRESHOLD;
    }

    private static void stopAll(Mecanum drivetrain, DcMotorEx[] wheels) {
        drivetrain.stop();
        for (DcMotorEx wheel : wheels) wheel.setPower(0);
    }
}
