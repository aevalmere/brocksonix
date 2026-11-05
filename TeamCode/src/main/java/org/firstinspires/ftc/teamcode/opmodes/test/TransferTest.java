package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.hardware.BulkReads;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Transfer;

/**
 * Drive around while checking both ends of the rail. It builds Pedro's Mecanum
 * drivetrain and the Intake and Transfer subsystems: no Robot or break beams.
 * The configuration needs the four drive motors, "intake", and "transfer".
 *
 * Sticks drive robot-centric with Drive Test's stick code
 * (DriveTest.stickPowers). Right trigger (hold): slow mode. From Start, both
 * motors run at their collect powers. Left trigger (hold): reverse both.
 * Transfer's hardware direction is FORWARD and Transfer.REVERSED defaults false.
 */
@TeleOp(name = "Transfer Test", group = "Test")
public class TransferTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Telemetry screen = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        BulkReads bulkReads = new BulkReads(hardwareMap);
        // Moves nothing: Mecanum only sets directions and float mode.
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);

        // Use the match subsystems so this checks their motor directions and power settings.
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);

        screen.addLine("Ready. Intake and transfer start spinning at Start, with no button.");
        screen.update();

        waitForStart();
        // So a "WasPressed" button added later can't fire on a press made during init.
        gamepad1.resetEdgeDetection();
        while (opModeIsActive()) {
            bulkReads.clear();

            // true = driver input, so the wheels brake at 0 power (as in Drive Test).
            drivetrain.drive(DriveTest.stickPowers(gamepad1), true);

            boolean reversing = gamepad1.left_trigger > MainTeleOp.TRIGGER_THRESHOLD;
            if (reversing) {
                intake.reverse();
                transfer.reverse();
            } else {
                intake.collect(false);
                transfer.collect(false);
            }

            screen.addData("Intake", "%s at %.2f power",
                    reversing ? "reversing (LT)" : "running", reversing ? Intake.REVERSE_POWER : Intake.COLLECT_POWER);
            screen.addData("Transfer", "%s at %.2f power",
                    reversing ? "reversing (LT)" : "running", reversing ? Transfer.REVERSE_POWER : Transfer.COLLECT_POWER);
            screen.addData("Slow mode (RT)", DriveTest.slowHeld(gamepad1) ? "on" : "off");
            screen.addLine("Sticks: drive. RT (hold): slow. No button: both motors run. LT (hold): reverse both.");
            screen.update();
        }
        drivetrain.stop();
        intake.stop();
        transfer.stop();
    }
}
