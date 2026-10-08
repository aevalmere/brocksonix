package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.driver.AllianceSelector;
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotState;
import org.firstinspires.ftc.teamcode.robot.commands.RobotCommands;

/**
 * A skeleton to copy for a real autonomous. It has one placeholder in each
 * section (poses, init, start, routine, loop): replace them with the real thing.
 *
 * Robot.update() saves the trusted pose every loop, so Main TeleOp starts with
 * the right pose and alliance without relocalizing. preselectTeleOp queues up
 * Main TeleOp on the Driver Station for when auto ends.
 */
@Configurable
@Autonomous(name = "Auto Template", preselectTeleOp = "Main TeleOp")
// TODO(9): Copy this file for a real auto, fill in the poses and the routine, and delete the @Disabled line.
@Disabled
public class AutoTemplate extends LinearOpMode {
    private static final double TELEMETRY_INTERVAL_MS = 200;

    // ---- 1. Poses: written for red, flipped for blue ----
    // Inches, Pedro frame (see field/Field.java). Heading is in degrees, 0 faces +x, counter-clockwise is positive.
    // These are plain numbers so Panels can edit them. runOpMode builds the Poses after init, so an edit applies to the next run.
    // TODO(9): Measure where the robot starts on the field. Center of the turret, with the robot sitting in place.
    public static double START_X = 8;
    public static double START_Y = 24;
    public static double START_HEADING_DEG = 180;
    // TODO(9): Pick where to shoot from and check the distance to the cell is inside the ShotTables range.
    public static double SHOOT_X = 36;
    public static double SHOOT_Y = 48;
    public static double SHOOT_HEADING_DEG = 180;

    @Override
    public void runOpMode() {
        // ---- 2. Init ----
        Telemetry screen = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        Scheduler.reset();
        Robot robot = new Robot(hardwareMap);
        RobotCommands commands = new RobotCommands(robot);
        AllianceSelector allianceSelector = new AllianceSelector(gamepad1, gamepad2);

        while (opModeInInit()) {
            allianceSelector.update(screen);
            if (RobotState.alliance == null) {
                screen.addLine("!! PICK AN ALLIANCE (D-pad up). Auto does nothing without one.");
            }
            screen.update();
        }

        // ---- 3. Start ----
        // Shooting at the wrong goal is worse than sitting still.
        if (RobotState.alliance == null) {
            while (opModeIsActive()) {
                screen.addLine("!! NO ALLIANCE PICKED: auto is doing nothing. Stop, restart the OpMode and pick one in init.");
                screen.update();
                sleep((long) TELEMETRY_INTERVAL_MS);
            }
            return;
        }
        Pose start = new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
        robot.setStartPose(start);

        // ---- 4. Routine ----
        // The other building blocks (follow, shootAll, shootContinuous, sweepAndShoot, ...) are in RobotCommands.
        // To retry anything else until a check passes, wrap it in a Retry (robot/commands/Retry.java).
        Pose shootSpot = new Pose(SHOOT_X, SHOOT_Y, Math.toRadians(SHOOT_HEADING_DEG));
        // TODO(9): Replace this placeholder with the real routine.
        Scheduler.schedule(sequential(
                commands.driveTo(shootSpot),
                commands.shootAllWithRetry()
        ));

        // ---- 5. Loop ----
        ElapsedTime sinceTelemetry = new ElapsedTime();
        while (opModeIsActive()) {
            // Commands set what the robot should do, then update applies it in this same loop.
            Scheduler.execute();
            robot.update();

            // MainTeleOp.FULL_TELEMETRY switches the auto's telemetry too.
            if (MainTeleOp.FULL_TELEMETRY || sinceTelemetry.milliseconds() >= TELEMETRY_INTERVAL_MS) {
                sinceTelemetry.reset();
                robot.addWarnings(screen);
                if (MainTeleOp.FULL_TELEMETRY) robot.addTelemetry(screen);
                screen.update();
            }
        }
        // Auto usually ends in the middle of a drive. This puts back any path speed limit,
        // so it can't slow down the next OpMode.
        robot.stop();
    }
}
