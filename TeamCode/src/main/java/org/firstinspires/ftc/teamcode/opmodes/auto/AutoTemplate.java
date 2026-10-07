package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.AllianceSelector;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotCommands;
import org.firstinspires.ftc.teamcode.robot.RobotState;

/**
 * A skeleton to copy for a real autonomous. It has one placeholder in each
 * section (poses, init, start, routine, loop): replace them with the real thing.
 *
 * Robot.update() saves the trusted pose every loop, so Main TeleOp starts with
 * the right pose and alliance without relocalizing. preselectTeleOp queues up
 * Main TeleOp on the Driver Station for when auto ends.
 */
@Autonomous(name = "Auto Template", preselectTeleOp = "Main TeleOp")
// TODO(12): Copy this file for a real auto, fill in the poses and the routine, and delete the @Disabled line.
@Disabled
public class AutoTemplate extends LinearOpMode {
    private static final double TELEMETRY_INTERVAL_MS = 200;

    // ---- 1. Poses: written for red, flipped for blue ----
    // Inches, Pedro frame (see field/Field.java). Heading is in radians, 0 faces +x.
    // TODO(12): Measure where the robot starts on the field. Center of the turret, with the robot sitting in place.
    private static final Pose START = new Pose(8, 24, Math.PI);
    // TODO(12): Pick where to shoot from and check the distance to the cell is inside the ShotTables range.
    private static final Pose SHOOT_SPOT = new Pose(36, 48, Math.PI);

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
        robot.setStartPose(START);

        // ---- 4. Routine ----
        // The other building blocks (follow, shootContinuous, sweepAndShoot, ...) are in RobotCommands.
        // TODO(12): Replace this placeholder with the real routine.
        Scheduler.schedule(sequential(
                commands.driveTo(SHOOT_SPOT),
                commands.shootAll()
        ));

        // ---- 5. Loop ----
        ElapsedTime sinceTelemetry = new ElapsedTime();
        while (opModeIsActive()) {
            // Commands set what the robot should do, then update applies it in this same loop.
            Scheduler.execute();
            robot.update();

            if (sinceTelemetry.milliseconds() >= TELEMETRY_INTERVAL_MS) {
                sinceTelemetry.reset();
                robot.addWarnings(screen);
                robot.addTelemetry(screen);
                screen.update();
            }
        }
    }
}
