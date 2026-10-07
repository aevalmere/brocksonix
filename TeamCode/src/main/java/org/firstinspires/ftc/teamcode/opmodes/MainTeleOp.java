package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Corner;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.robot.Feedback;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotState;

/**
 * Driver 1 (gamepad1) drives, shoots, passes and intakes.
 * Driver 2 (gamepad2) handles things that are easy to watch from the side:
 * aim trims and telemetry. The target cell is picked automatically from
 * which half of the field the robot is on.
 *
 * The full button map is in TELEOP_PLAN.md.
 */
@Configurable
@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends LinearOpMode {
    // TODO(10): Watch Loop ms with everything running, goal under 15. Check again with telemetry off (gamepad2 Options).
    public static double TELEMETRY_INTERVAL_MS = 200;
    public static double TURRET_TRIM_STEP_DEG = 2;
    public static double RPM_TRIM_STEP = 25;
    /** How far RT (pass) and LT (unjam) must be pulled to count as pressed. */
    public static double TRIGGER_THRESHOLD = 0.5;

    private Robot robot;
    private boolean telemetryOn = true;

    @Override
    public void runOpMode() {
        Telemetry screen = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        Scheduler.reset();
        robot = new Robot(hardwareMap);
        Feedback feedback = new Feedback(gamepad1);

        AllianceSelector allianceSelector = new AllianceSelector(gamepad1, gamepad2);
        Pose savedPose = RobotState.recentPose();
        Alliance savedAlliance = RobotState.alliance;

        while (opModeInInit()) {
            allianceSelector.update(screen);
            // The saved pose is in the old alliance's frame, so switching drops it.
            if (RobotState.alliance != savedAlliance) savedPose = null;
            screen.addData("Start pose", savedPose == null
                    ? "unknown: relocalize in a corner before shooting"
                    : "from the last OpMode " + savedPose);
            screen.update();
        }

        if (savedPose != null) {
            robot.setPose(savedPose, true);
        } else if (RobotState.alliance != null) {
            robot.setPose(Field.cornerPose(RobotState.alliance, Corner.RIGHT_INTAKE_TO_DRIVER), false);
        }

        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime sinceTelemetry = new ElapsedTime();
        while (opModeIsActive()) {
            double loopMs = loopTimer.milliseconds();
            loopTimer.reset();

            driverOne(gamepad1);
            driverTwo(gamepad2);
            robot.update();
            Scheduler.execute();
            feedback.update(robot.storage, robot.warnings());

            if (sinceTelemetry.milliseconds() >= TELEMETRY_INTERVAL_MS) {
                sinceTelemetry.reset();
                robot.addWarnings(screen);
                screen.addData("Loop", "%.1f ms", loopMs);
                if (telemetryOn) robot.addTelemetry(screen);
                screen.update();
            }
        }
        feedback.stop();
    }

    private void driverOne(Gamepad pad) {
        robot.drive.drive(
                -pad.left_stick_y,
                -pad.left_stick_x,
                -pad.right_stick_x,
                RobotState.alliance != null ? RobotState.alliance : Alliance.RED);

        robot.shootHeld = pad.right_bumper;
        robot.passHeld = pad.right_trigger > TRIGGER_THRESHOLD;
        robot.unjamHeld = pad.left_trigger > TRIGGER_THRESHOLD;

        if (pad.leftBumperWasPressed()) robot.flowerIntake.toggle();

        if (pad.dpadRightWasPressed()) robot.relocalize(Corner.RIGHT_INTAKE_TO_DRIVER);
        if (pad.dpadLeftWasPressed()) robot.relocalize(Corner.LEFT_INTAKE_TO_DRIVER);
        if (pad.circleWasPressed()) robot.relocalize(Corner.RIGHT_INTAKE_TO_SIDE_WALL);
        if (pad.squareWasPressed()) robot.relocalize(Corner.LEFT_INTAKE_TO_SIDE_WALL);
    }

    private void driverTwo(Gamepad pad) {
        if (pad.dpadLeftWasPressed()) robot.turret.addTrim(TURRET_TRIM_STEP_DEG);
        if (pad.dpadRightWasPressed()) robot.turret.addTrim(-TURRET_TRIM_STEP_DEG);
        if (pad.dpadUpWasPressed()) robot.shooter.addTrim(RPM_TRIM_STEP);
        if (pad.dpadDownWasPressed()) robot.shooter.addTrim(-RPM_TRIM_STEP);

        if (pad.optionsWasPressed()) telemetryOn = !telemetryOn;
    }
}
