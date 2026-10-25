package org.firstinspires.ftc.teamcode.opmodes.teleop;

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
import org.firstinspires.ftc.teamcode.driver.AllianceSelector;
import org.firstinspires.ftc.teamcode.driver.Feedback;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Corner;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotState;

/**
 * Driver 1 (gamepad1) drives, shoots, passes and intakes.
 * Driver 2 (gamepad2) handles things that are easy to watch from the side:
 * aim trims. The target cell is picked automatically from which half of
 * the field the robot is on.
 *
 * The full button map is in TELEOP_PLAN.md.
 */
@Configurable
@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends LinearOpMode {
    // TODO(10): Set to false before competition.
    /**
     * true: every value, every loop, so Panels shows live values while tuning.
     * false: only the warnings and loop time, every TELEMETRY_INTERVAL_MS. Autos use this too.
     */
    public static boolean FULL_TELEMETRY = true;
    // TODO(12): Watch Loop ms with everything running, goal under 15. Check again with FULL_TELEMETRY false.
    public static double TELEMETRY_INTERVAL_MS = 200;
    // TODO(12): Driver 2 practice: one press should fix a typical miss. Raise if it takes several presses, lower if one press overshoots.
    public static double TURRET_TRIM_STEP_DEG = 2;
    // TODO(12): Same as the turret trim, for shots that land short or long (static hood).
    public static double RPM_TRIM_STEP = 25;
    // TODO(16): Main TeleOp, COAX: one press should fix a typical high or low miss. Raise if it takes several presses, lower if one overshoots.
    /** Coax hood: D-pad up/down trims the hood angle instead of the RPM, since the hood re-solves for any RPM. */
    public static double HOOD_TRIM_STEP_DEG = 0.5;
    // TODO(12): Raise it if a resting finger fires RT or LT; lower it if a full pull doesn't register.
    /** How far RT (pass) and LT (unjam) must be pulled to count as pressed. */
    public static double TRIGGER_THRESHOLD = 0.5;

    private Robot robot;

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
            // Without a saved pose, field-centric driving uses a guessed heading: intake facing the driver.
            screen.addData("Start pose", savedPose == null
                    ? "unknown: set the robot down with the intake facing you, and relocalize in a corner before shooting"
                    : "from the last OpMode " + savedPose);
            screen.update();
        }
        // "WasPressed" remembers presses made during init, which would fire on the first loop
        // (a relocalize, an intake toggle, a trim). Start with none.
        gamepad1.resetEdgeDetection();
        gamepad2.resetEdgeDetection();

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

            // Same order as auto: buttons, then commands, then update applies both in this loop.
            driverOne(gamepad1);
            driverTwo(gamepad2);
            Scheduler.execute();
            robot.update();
            feedback.update(robot.storage, robot.warnings());

            if (FULL_TELEMETRY || sinceTelemetry.milliseconds() >= TELEMETRY_INTERVAL_MS) {
                sinceTelemetry.reset();
                robot.addWarnings(screen);
                screen.addData("Loop", "%.1f ms", loopMs);
                if (FULL_TELEMETRY) robot.addTelemetry(screen);
                screen.update();
            }
        }
        feedback.stop();
        robot.stop();
    }

    private void driverOne(Gamepad pad) {
        // Before an alliance is picked, drive as red: driving still works, and nothing fires without an alliance.
        robot.drive.drive(
                -pad.left_stick_y,
                -pad.left_stick_x,
                -pad.right_stick_x,
                RobotState.alliance != null ? RobotState.alliance : Alliance.RED);

        // These are set from the buttons every loop. A macro that sets one of them too
        // has to be OR-ed in here, or the button overwrites it on the next loop.
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
        // With the coax hood an RPM trim does nothing useful: the hood just re-solves for the new speed.
        // So up/down trims the hood there. Steeper mostly means higher at the cell.
        if (robot.usingCoaxHood()) {
            if (pad.dpadUpWasPressed()) robot.coaxHood.addTrim(HOOD_TRIM_STEP_DEG);
            if (pad.dpadDownWasPressed()) robot.coaxHood.addTrim(-HOOD_TRIM_STEP_DEG);
        } else {
            if (pad.dpadUpWasPressed()) robot.shooter.addTrim(RPM_TRIM_STEP);
            if (pad.dpadDownWasPressed()) robot.shooter.addTrim(-RPM_TRIM_STEP);
        }
    }
}
