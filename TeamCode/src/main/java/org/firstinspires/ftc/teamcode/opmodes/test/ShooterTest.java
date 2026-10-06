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
 */
@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTest extends LinearOpMode {
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
            shooter.setTargetRpm(on ? rpm : 0);
            shooter.setBoost(fire);
            if (fire) door.open();
            else door.close();

            if (fire && door.isFullyOpen()) rail.feed(1.0);
            else if (fire) rail.stop();
            else rail.collect(storage.slot1Occupied(), storage.isFull());

            shooter.update();
            door.update();

            screen.addData("Wheel (Cross)", on ? "on" : "off");
            screen.addData("At speed", shooter.atSpeed());
            shooter.addTelemetry(screen);
            storage.addTelemetry(screen);
            screen.addData("Battery", "%.2f V", voltage.get());
            screen.update();
        }
    }
}
