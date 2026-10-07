package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Door;
import org.firstinspires.ftc.teamcode.subsystems.Rail;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Storage;
import org.firstinspires.ftc.teamcode.util.BulkReads;
import org.firstinspires.ftc.teamcode.util.VoltageCache;

/**
 * Flywheel tuning and filling the shot tables. Put the robot a measured
 * distance from the cell, find the RPM that scores, write the pair into
 * ShotTables. Tune KS/KV/KP live in Panels (Shooter).
 *
 * Cross: wheel on/off. D-pad up/down: ±100 RPM, left/right: ±25.
 * Right bumper (hold): open the door and feed. Rail collects otherwise.
 *
 * Before the first spin, with the wheel off: hold Square to run only the left
 * motor, Circle to run only the right one, at low power. Each must push the
 * shooting direction; if not, flip its REVERSED flag in Panels (works live).
 * With the left motor running, RPM must read positive, or flip ENCODER_REVERSED.
 */
@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTest extends LinearOpMode {
    private static final double MOTOR_TEST_POWER = 0.2;

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
            storage.update();

            if (gamepad1.crossWasPressed()) on = !on;
            if (gamepad1.dpadUpWasPressed()) rpm += 100;
            if (gamepad1.dpadDownWasPressed()) rpm -= 100;
            if (gamepad1.dpadRightWasPressed()) rpm += 25;
            if (gamepad1.dpadLeftWasPressed()) rpm -= 25;

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

            if (fire && door.isFullyOpen()) rail.feed(1.0);
            else if (fire) rail.stop();
            else rail.collect(storage.slot1Occupied(), storage.isFull());

            shooter.update();
            door.update();

            screen.addData("Wheel (Cross)", on ? "on" : "off");
            screen.addData("One motor (hold, wheel off)", "Square = left, Circle = right, %.1f power", MOTOR_TEST_POWER);
            screen.addData("At speed", shooter.atSpeed());
            shooter.addTelemetry(screen);
            storage.addTelemetry(screen);
            screen.addData("Battery", "%.2f V", voltage.get());
            screen.update();
        }
    }
}
