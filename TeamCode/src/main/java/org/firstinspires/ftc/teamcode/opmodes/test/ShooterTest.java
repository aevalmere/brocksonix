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
import org.firstinspires.ftc.teamcode.robot.subsystems.Door;
import org.firstinspires.ftc.teamcode.robot.subsystems.Rail;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Storage;

/**
 * Flywheel tuning and filling the shot tables. Put the robot a measured
 * distance from the cell, find the RPM that scores, put the pair into
 * ShotTables (also live in Panels). Tune KS/KV/KP live in Panels (Shooter).
 *
 * Cross: wheel on/off. D-pad up/down: big RPM step, left/right: small step.
 * Right bumper (hold): open the door and feed. Rail collects otherwise.
 *
 * Before the first spin, with the wheel off: hold Square to run only the left
 * motor, Circle to run only the right one, at low power. Each must push the
 * shooting direction; if not, flip its REVERSED flag in Panels (works live).
 * With the left motor running, RPM must read positive, or flip ENCODER_REVERSED.
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
        Rail rail = new Rail(hardwareMap);
        Storage storage = new Storage(hardwareMap);
        boolean on = false;
        double rpm = 3000;

        waitForStart();
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
                shooter.setOpenLoopTest(testLeft ? MOTOR_TEST_POWER : 0, testRight ? MOTOR_TEST_POWER : 0);
            } else {
                shooter.setTargetRpm(on ? rpm : 0);
            }
            shooter.setBoost(fire);
            if (fire) door.open();
            else door.close();

            if (fire && door.isFullyOpen()) rail.feed(FEED_POWER);
            else if (fire) rail.stop();
            else rail.collect(storage.slot1Occupied(), storage.isFull());

            shooter.update();
            door.update();

            String tableProblem = ShotTables.orderProblem();
            if (tableProblem != null) screen.addLine("!! " + tableProblem + " (fix in Panels, ShotTables)");
            screen.addData("Wheel (Cross)", on ? "on" : "off");
            screen.addData("One motor (hold, wheel off)", "Square = left, Circle = right, %.2f power", MOTOR_TEST_POWER);
            screen.addData("At speed", shooter.atSpeed());
            shooter.addTelemetry(screen);
            storage.addTelemetry(screen);
            screen.addData("Battery", "%.2f V", voltage.get());
            screen.update();
        }
    }
}
