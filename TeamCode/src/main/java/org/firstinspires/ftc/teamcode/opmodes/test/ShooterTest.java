package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.hardware.BulkReads;
import org.firstinspires.ftc.teamcode.robot.hardware.VoltageCache;
import org.firstinspires.ftc.teamcode.robot.shot.ShotTables;
import org.firstinspires.ftc.teamcode.robot.subsystems.CoaxHood;
import org.firstinspires.ftc.teamcode.robot.subsystems.Door;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Storage;
import org.firstinspires.ftc.teamcode.robot.subsystems.Transfer;

/**
 * Flywheel tuning and filling the shot tables. Put the robot a measured
 * distance from the cell, find the RPM that scores, put the pair into
 * ShotTables (also live in Panels). Tune KS/KV/KP live in Panels (Shooter).
 *
 * Cross: wheel on/off. D-pad up/down: big RPM step, left/right: small step.
 * Right bumper (hold): open the door, and feed once it is fully open, at any
 * wheel speed. Letting go closes the door and collects in the same loop. Main
 * TeleOp (FireControl) keeps the door open DOOR_CLOSE_DELAY_MS longer and only
 * collects once it is closed (Door.CLOSE_TIME_MS).
 *
 * Before the first spin, with the wheel off: hold Square to run only the left
 * motor, Circle to run only the right one, at low power. Each must push the
 * shooting direction; if not, flip its REVERSED flag in Panels (works live).
 * With the left motor running, RPM must read positive, or flip ENCODER_REVERSED.
 *
 * With the coax hood on the robot, it holds CoaxHood.STATIC_LAUNCH_DEG, like Main
 * TeleOp with Robot.HOOD = STATIC, so the SHOT_RPM rows found here fit the static tables.
 */
@Configurable
@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTest extends LinearOpMode {
    /** Power for the one-motor check (Square / Circle). Keep it low. */
    public static double MOTOR_TEST_POWER = 0.2;
    /** D-pad up/down changes the target RPM by this much. */
    public static double RPM_STEP_BIG = 100;
    /** D-pad left/right changes the target RPM by this much. */
    public static double RPM_STEP_SMALL = 25;
    /** Rail power while feeding (right bumper). Compare with ShotTables.SHOT_FEED_POWER. */
    public static double FEED_POWER = 1.0;

    @Override
    public void runOpMode() {
        Telemetry screen = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        BulkReads bulkReads = new BulkReads(hardwareMap);
        VoltageCache voltage = new VoltageCache(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap, voltage);
        Door door = new Door(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Storage storage = new Storage(hardwareMap);
        // Optional, like in Robot: null when the configuration has no hood servo.
        CoaxHood hood = CoaxHood.isConfigured(hardwareMap) ? new CoaxHood(hardwareMap) : null;
        boolean on = false;
        double rpm = 3000;

        waitForStart();
        // "WasPressed" remembers presses made during init: Cross would turn the wheel on at start.
        gamepad1.resetEdgeDetection();
        while (opModeIsActive()) {
            bulkReads.clear();
            shooter.readSensors();
            storage.update();

            if (gamepad1.crossWasPressed()) on = !on;
            if (gamepad1.dpadUpWasPressed()) rpm += RPM_STEP_BIG;
            if (gamepad1.dpadDownWasPressed()) rpm -= RPM_STEP_BIG;
            if (gamepad1.dpadRightWasPressed()) rpm += RPM_STEP_SMALL;
            if (gamepad1.dpadLeftWasPressed()) rpm -= RPM_STEP_SMALL;

            boolean fire = gamepad1.right_bumper;
            // One motor alone, open loop. Only with the wheel off, so it can't fight the closed loop.
            boolean testLeft = !on && gamepad1.square;
            boolean testRight = !on && gamepad1.circle;
            if (testLeft || testRight) {
                shooter.setTestPowers(testLeft ? MOTOR_TEST_POWER : 0, testRight ? MOTOR_TEST_POWER : 0);
            } else {
                shooter.setTargetRpm(on ? rpm : 0);
            }
            shooter.setBoost(fire);
            if (fire) door.open();
            else door.close();

            if (fire && door.isFullyOpen()) {
                intake.feed(FEED_POWER);
                transfer.feed(FEED_POWER);
            } else if (fire) {
                intake.stop();
                transfer.stop();
            } else {
                intake.collect(storage.isFull());
                transfer.collect(storage.slot1Occupied());
            }

            shooter.update();
            door.update();
            if (hood != null) {
                // Every loop, so a Panels edit to STATIC_LAUNCH_DEG moves it live.
                hood.setTargetDeg(CoaxHood.STATIC_LAUNCH_DEG);
                hood.update();
            }

            String tableProblem = ShotTables.orderProblem();
            if (tableProblem != null) screen.addLine("!! " + tableProblem + " (fix in Panels, ShotTables)");
            screen.addData("Wheel (Cross)", on ? "on" : "off");
            screen.addData("One motor (hold, wheel off)", "Square = left, Circle = right, %.2f power", MOTOR_TEST_POWER);
            screen.addData("At speed", shooter.atSpeed());
            shooter.addTelemetry(screen);
            storage.addTelemetry(screen);
            if (hood != null) {
                screen.addLine("Coax hood: holding STATIC_LAUNCH_DEG. Wait for \"at target\" before firing.");
                hood.addTelemetry(screen);
            }
            screen.addData("Battery", "%.2f V", voltage.get());
            screen.update();
        }
    }
}
