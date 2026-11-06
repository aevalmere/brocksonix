package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.hardware.BulkReads;
import org.firstinspires.ftc.teamcode.robot.hardware.VoltageCache;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Angles;

/**
 * Turret setup and tuning. Change the gains live in Panels (page "Turret") and the
 * target here (page "TurretTest"), and graph "target" against "angle" in Panels.
 *
 * Cross: switch between manual (left stick X drives the servo, up to
 *   Turret.MAX_POWER) and aiming.
 * Aiming:
 *   TARGET_DEG (Panels) is the target. D-pad sets it to 0 (up), 90 (left), -90 (right),
 *   180 (down); bumpers nudge it ±10°.
 *   Square: step test. The target jumps between TARGET_DEG and TARGET_DEG + STEP_DEG every
 *     STEP_MS, for tuning the P and D gains and KS.
 *   Triangle: sweep. The target turns at SWEEP_DEG_PER_SEC, like the robot turning under
 *     the turret, for tuning the KV feedforward without driving. Stopping the sweep keeps
 *     the target where it got to.
 */
@Configurable
@TeleOp(name = "Turret Test", group = "Test")
public class TurretTest extends LinearOpMode {
    /** The aiming target, degrees: 0 = straight out the front, counter-clockwise positive. */
    public static double TARGET_DEG = 0;
    /** Step test: how far the target jumps, and how long it stays at each end. */
    public static double STEP_DEG = 90;
    public static double STEP_MS = 1500;
    /** Sweep: how fast the target turns. Negative turns clockwise. */
    public static double SWEEP_DEG_PER_SEC = 180;

    @Override
    public void runOpMode() {
        Telemetry screen = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        BulkReads bulkReads = new BulkReads(hardwareMap);
        Turret turret = new Turret(hardwareMap, new VoltageCache(hardwareMap));
        boolean manual = true;
        boolean stepping = false;
        boolean sweeping = false;
        ElapsedTime stepTimer = new ElapsedTime();
        ElapsedTime loopTimer = new ElapsedTime();

        waitForStart();
        // "WasPressed" remembers presses made during init, which would fire on the first loop.
        gamepad1.resetEdgeDetection();
        stepTimer.reset();
        loopTimer.reset();
        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();
            bulkReads.clear();
            turret.readSensors();

            if (gamepad1.crossWasPressed()) manual = !manual;
            if (gamepad1.dpadUpWasPressed()) TARGET_DEG = 0;
            if (gamepad1.dpadLeftWasPressed()) TARGET_DEG = 90;
            if (gamepad1.dpadRightWasPressed()) TARGET_DEG = -90;
            if (gamepad1.dpadDownWasPressed()) TARGET_DEG = 180;
            if (gamepad1.leftBumperWasPressed()) TARGET_DEG += 10;
            if (gamepad1.rightBumperWasPressed()) TARGET_DEG -= 10;
            if (gamepad1.squareWasPressed()) {
                stepping = !stepping;
                sweeping = false;
                stepTimer.reset();
            }
            if (gamepad1.triangleWasPressed()) {
                sweeping = !sweeping;
                stepping = false;
            }

            if (sweeping && !manual) TARGET_DEG += SWEEP_DEG_PER_SEC * dt;
            TARGET_DEG = Angles.wrapDegrees(TARGET_DEG);

            double target = TARGET_DEG;
            // Even half-periods sit at TARGET_DEG, odd ones at TARGET_DEG + STEP_DEG.
            if (stepping && (long) (stepTimer.milliseconds() / Math.max(STEP_MS, 100)) % 2 == 1) {
                target += STEP_DEG;
            }

            // Capped like aiming mode, so the stick never pushes harder than MAX_POWER.
            double stick = Range.clip(-gamepad1.left_stick_x, -Turret.MAX_POWER, Turret.MAX_POWER);
            turret.setTestPower(manual ? (Double) stick : null);
            turret.setTargetDeg(target);
            turret.update();

            String test = stepping ? "step (Square)" : sweeping ? "sweep (Triangle)" : "hold";
            screen.addData("Mode (Cross)", manual ? "manual" : "aiming, " + test);
            screen.addData("Encoder volts", "%.3f", turret.encoderVolts());
            // Wrapped to -180..180 like every other angle, so it reads the same number ENCODER_ZERO_DEG holds.
            screen.addData("Turret raw encoder deg", "%.1f", Angles.wrapDegrees(turret.rawEncoderDeg()));
            turret.addTelemetry(screen);
            // Plain numbers, for the Panels graph.
            screen.addData("target", turret.aimDeg());
            screen.addData("angle", turret.angleDeg());
            screen.addData("error", turret.errorDeg());
            screen.addData("power", turret.power());
            screen.update();
        }
    }
}
