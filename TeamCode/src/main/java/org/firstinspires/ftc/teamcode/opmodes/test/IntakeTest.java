package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.hardware.BulkReads;
import org.firstinspires.ftc.teamcode.robot.hardware.CachedMotor;
import org.firstinspires.ftc.teamcode.robot.hardware.HardwareNames;
import org.firstinspires.ftc.teamcode.robot.subsystems.Rail;
import org.firstinspires.ftc.teamcode.robot.subsystems.Storage;
import org.firstinspires.ftc.teamcode.util.Debouncer;

/**
 * Drive around and pick balls up. It builds only Pedro's Mecanum drivetrain
 * (from Constants.drivetrainConfig) and the intake motor: no Pinpoint, no
 * Follower, no Robot. The configuration needs the four drive motors and
 * "intake": no transfer motor, no beams.
 *
 * Sticks drive robot-centric with Drive Test's stick code
 * (DriveTest.stickPowers). Right trigger (hold): slow mode, all three sticks
 * times DriveTest.SLOW_SCALE (Panels page "DriveTest").
 *
 * From Start the intake runs at Rail.INTAKE_POWER with no button. Balls must
 * move in, toward the door. If they go out, flip Rail.INTAKE_REVERSED (Panels
 * page "Rail").
 *
 * Left trigger (hold): reverse at Rail.REVERSE_POWER, to back balls out, even
 * when the beam reads full.
 *
 * The full beam ("beamFull") is optional. Without it the intake runs nonstop.
 * With it, the intake stops while the beam reads full (blocked for
 * Storage.FULL_HOLD_MS, as in Storage) and starts again when it clears. Block
 * it by hand to check. If it reads blocked with nothing in front, flip
 * Storage.BLOCKED_READS_LOW.
 */
@TeleOp(name = "Intake Test", group = "Test")
public class IntakeTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Telemetry screen = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        BulkReads bulkReads = new BulkReads(hardwareMap);
        // Moves nothing: Mecanum only sets directions and float mode.
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);

        // Same setup as Rail's motors. The motor stays FORWARD: INTAKE_REVERSED is applied
        // when the power is written, so flipping it in Panels works live.
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, HardwareNames.INTAKE_MOTOR);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        CachedMotor intake = new CachedMotor(intakeMotor);

        // tryGet gives null instead of an error when the beam isn't in the configuration.
        DigitalChannel fullBeam = hardwareMap.tryGet(DigitalChannel.class, HardwareNames.BEAM_FULL);
        if (fullBeam != null) fullBeam.setMode(DigitalChannel.Mode.INPUT);
        Debouncer blocked = new Debouncer(false);
        ElapsedTime blockedFor = new ElapsedTime();

        screen.addLine("Ready. The intake starts spinning at Start, with no button.");
        screen.addData("Full beam", fullBeam == null ? "not in config, so the intake runs nonstop" : "found");
        screen.update();

        waitForStart();
        // So a "WasPressed" button added later can't fire on a press made during init.
        gamepad1.resetEdgeDetection();
        while (opModeIsActive()) {
            bulkReads.clear();

            // true = driver input, so the wheels brake at 0 power (as in Drive Test).
            drivetrain.drive(DriveTest.stickPowers(gamepad1), true);

            boolean full = false;
            String beam;
            if (fullBeam == null) {
                beam = "not in config, so the intake runs nonstop";
            } else {
                // Storage's rule: a ball passing through blocks the beam briefly, so full
                // means blocked (after the debounce) for at least FULL_HOLD_MS.
                boolean wasBlocked = blocked.get();
                boolean isBlocked = blocked.update(fullBeam.getState() != Storage.BLOCKED_READS_LOW, Storage.DEBOUNCE_MS);
                if (isBlocked && !wasBlocked) blockedFor.reset();
                full = isBlocked && blockedFor.milliseconds() >= Storage.FULL_HOLD_MS;
                if (full) beam = "full";
                else if (isBlocked) beam = "blocked";
                else beam = "clear";
            }

            // Reverse beats a full beam, so balls can always be backed out.
            double power;
            String doing;
            if (gamepad1.left_trigger > MainTeleOp.TRIGGER_THRESHOLD) {
                power = Rail.REVERSE_POWER;
                doing = "reversing (LT)";
            } else if (full) {
                power = 0;
                doing = "stopped (beam full)";
            } else {
                power = Rail.INTAKE_POWER;
                doing = "running";
            }
            intake.setPower(Rail.INTAKE_REVERSED ? -power : power);

            screen.addData("Intake", "%s at %.2f power", doing, power);
            screen.addData("Full beam", beam);
            screen.addData("Slow mode (RT)", DriveTest.slowHeld(gamepad1) ? "on" : "off");
            screen.addLine("Sticks: drive. RT (hold): slow. No button: intake runs. LT (hold): reverse.");
            screen.update();
        }
        drivetrain.stop();
        intake.setPower(0);
    }
}
