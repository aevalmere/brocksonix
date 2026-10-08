package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.hardware.BulkReads;
import org.firstinspires.ftc.teamcode.robot.hardware.VoltageCache;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;

/**
 * Turret setup and tuning. Change gains live in Panels (Turret) and watch the
 * error settle.
 *
 * Cross: switch between manual (left stick X drives the servo) and aiming.
 * D-pad: aim at 0 (up), 90 (left), -90 (right), 180 (down).
 * Bumpers: nudge the target ±10°.
 */
@TeleOp(name = "Turret Test", group = "Test")
public class TurretTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Telemetry screen = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        BulkReads bulkReads = new BulkReads(hardwareMap);
        Turret turret = new Turret(hardwareMap, new VoltageCache(hardwareMap));
        boolean manual = true;
        double target = 0;

        waitForStart();
        while (opModeIsActive()) {
            bulkReads.clear();
            turret.readSensors();

            if (gamepad1.crossWasPressed()) manual = !manual;
            if (gamepad1.dpadUpWasPressed()) target = 0;
            if (gamepad1.dpadLeftWasPressed()) target = 90;
            if (gamepad1.dpadRightWasPressed()) target = -90;
            if (gamepad1.dpadDownWasPressed()) target = 180;
            if (gamepad1.leftBumperWasPressed()) target += 10;
            if (gamepad1.rightBumperWasPressed()) target -= 10;

            turret.setManualPower(manual ? (double) -gamepad1.left_stick_x : null);
            turret.setTargetDeg(target);
            turret.update();

            screen.addData("Mode (Cross)", manual ? "manual" : "aiming");
            screen.addData("Encoder volts", "%.3f", turret.encoderVolts());
            turret.addTelemetry(screen);
            screen.update();
        }
    }
}
