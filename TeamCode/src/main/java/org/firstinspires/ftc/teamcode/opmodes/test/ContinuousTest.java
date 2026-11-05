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
import org.firstinspires.ftc.teamcode.robot.hardware.VoltageCache;
import org.firstinspires.ftc.teamcode.robot.subsystems.Door;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Transfer;

/**
 * Continuous shooting with the real Intake, Transfer, Shooter, and Door
 * subsystems. The shooter targets 3000 RPM, the door stays open, and both rail
 * motors collect from Start. LT reverses the rail motors while the shooter
 * keeps spinning. No break beams or turret are needed.
 *
 * Before the first run, check each shooter motor at low power in Shooter Test.
 * Sticks drive robot-centric with Drive Test's stick code; RT (hold) is slow mode.
 */
@TeleOp(name = "Continuous Test", group = "Test")
public class ContinuousTest extends LinearOpMode {
    private static final double SHOOTER_TARGET_RPM = 3000;

    @Override
    public void runOpMode() {
        Telemetry screen = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        BulkReads bulkReads = new BulkReads(hardwareMap);
        VoltageCache voltage = new VoltageCache(hardwareMap);
        // Moves nothing: Mecanum only sets directions and float mode.
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);

        // Use the match subsystems so directions, control, and tuning stay consistent.
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap, voltage);
        Door door = new Door(hardwareMap);

        screen.addLine("Ready. Start spins the shooter at 3000 RPM, opens the door, and runs both rail motors.");
        screen.update();

        waitForStart();
        gamepad1.resetEdgeDetection();
        shooter.setTargetRpm(SHOOTER_TARGET_RPM);
        shooter.setBoost(true);

        while (opModeIsActive()) {
            bulkReads.clear();
            shooter.readSensors();

            drivetrain.drive(DriveTest.stickPowers(gamepad1), true);

            // Keep the gate open continuously so the rail can feed the spinning shooter.
            door.open();
            boolean reversing = gamepad1.left_trigger > MainTeleOp.TRIGGER_THRESHOLD;
            String railState;
            double intakePower;
            double transferPower;
            if (reversing) {
                intake.reverse();
                transfer.reverse();
                railState = "reversing (LT)";
                intakePower = Intake.REVERSE_POWER;
                transferPower = Transfer.REVERSE_POWER;
            } else if (door.isFullyOpen()) {
                intake.collect(false);
                transfer.collect(false);
                railState = "running";
                intakePower = Intake.COLLECT_POWER;
                transferPower = Transfer.COLLECT_POWER;
            } else {
                // Let the door finish opening before pushing balls into it.
                intake.stop();
                transfer.stop();
                railState = "waiting for door";
                intakePower = 0;
                transferPower = 0;
            }

            shooter.update();
            door.update();

            screen.addData("Intake", "%s at %.2f power",
                    railState, intakePower);
            screen.addData("Transfer", "%s at %.2f power",
                    railState, transferPower);
            screen.addData("Slow mode (RT)", DriveTest.slowHeld(gamepad1) ? "on" : "off");
            shooter.addTelemetry(screen);
            door.addTelemetry(screen);
            screen.addLine("Sticks: drive. RT: slow. LT: reverse intake and transfer. Shooter stays on; door stays open.");
            screen.update();
        }

        drivetrain.stop();
        intake.stop();
        transfer.stop();
        shooter.setTargetRpm(0);
        shooter.setBoost(false);
        shooter.update();
        door.close();
        door.update();
    }
}
