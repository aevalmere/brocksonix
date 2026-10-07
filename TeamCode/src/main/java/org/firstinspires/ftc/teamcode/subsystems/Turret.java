package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.HardwareNames;
import org.firstinspires.ftc.teamcode.util.Angles;
import org.firstinspires.ftc.teamcode.util.CachedCRServo;
import org.firstinspires.ftc.teamcode.util.VoltageCache;

/**
 * Infinite-rotation turret: an Axon in CR mode, geared so the turret turns
 * twice per servo turn, with an ELC absolute encoder on the turret itself.
 *
 * Angles are in degrees, relative to the robot: 0 = shooter points out the
 * robot's front, counter-clockwise positive. Because the turret spins
 * forever, it always takes the shortest way to the target.
 *
 * Control, all scaled for battery voltage:
 *   kP * error + kD * error rate     (two gain sets: FAR when the error is big, CLOSE to settle)
 *   + kS toward the target           (ramped in near zero so it doesn't buzz)
 *   + kV * how fast the target moves (cancels the robot turning underneath the turret)
 *
 * Safety: if the turret pushes at full power and the error doesn't shrink (encoder
 * unplugged or stuck, or SERVO_REVERSED and ENCODER_REVERSED disagree), a watchdog
 * cuts the power until the OpMode restarts. Ask it with watchdogTripped().
 */
@Configurable
public class Turret {
    // TODO(5): Turret Test, manual: turn slowly through full turns. Use the highest clean volts before it drops to 0.
    /** Encoder output at a full turn. The ELC's top end is noisy; Meta-Infinity used 3.2 V. Measure ours. */
    public static double ENCODER_FULL_TURN_VOLTS = 3.2;
    // TODO(5): Point the shooter straight out the front, read the angle, put it here.
    /** Raw encoder angle when the shooter points straight out the front. */
    public static double ENCODER_ZERO_DEG = 0;
    // TODO(5): The angle must increase when the turret turns counter-clockwise.
    public static boolean ENCODER_REVERSED = false;
    // TODO(5): Turret Test, manual: stick left must turn the turret counter-clockwise (seen from above).
    public static boolean SERVO_REVERSED = false;

    // TODO(5): Aiming mode, D-pad 0 to 180: tune FAR_KP and FAR_KD for fast moves with a small overshoot.
    public static double FAR_KP = 0.01;
    public static double FAR_KD = 0.0005;
    // TODO(5): Aiming mode, bumper nudges: tune CLOSE_KP and CLOSE_KD so it settles without buzzing.
    public static double CLOSE_KP = 0.006;
    public static double CLOSE_KD = 0.0003;
    // TODO(5): Aiming mode: if it chatters near the target, adjust this.
    /** Use CLOSE gains under this error; the band stops it flickering between sets. */
    public static double CLOSE_ZONE_DEG = 15;
    public static double CLOSE_ZONE_BAND_DEG = 2;

    // TODO(5): Manual mode: raise until the turret just starts to move, then use a bit less.
    public static double KS = 0.05;
    // TODO(5): Aiming mode: if it chatters near the target, adjust this. Higher = KS fades in over a wider zone.
    public static double KS_RAMP_DEG = 5;
    // TODO(5): Main TeleOp: spin the robot in place. Raise until the turret stops lagging behind the cell.
    /** Power per degree/second of target motion. */
    public static double KV = 0.0;
    // TODO(5): Lower if the gearing or wiring complains.
    public static double MAX_POWER = 0.8;

    // TODO(5): Largest aim error that still scores. Find it once the shooter works.
    public static double ON_TARGET_DEG = 3;

    // TODO(5): Aiming mode, D-pad 0 to 180: if a normal big move trips the watchdog, raise this time.
    /** The watchdog trips if the turret stays at full power this long and the error hasn't shrunk. */
    public static double WATCHDOG_MS = 1500;

    // Panels can set a tunable to 0; dividing by less than this would give NaN or Infinity.
    private static final double MIN_DIVISOR = 0.1;
    // "Full power" for the watchdog: this fraction of MAX_POWER or more.
    private static final double NEAR_MAX_POWER = 0.95;
    // The error must shrink by at least this much per watchdog window, or it counts as stuck.
    // The margin keeps a turret jittering around 180 degrees off from passing by luck.
    private static final double MIN_PROGRESS_DEG = 10;
    // If the target moves more than this during a watchdog window, the window doesn't count
    // (the robot is turning, so a stuck error is not proof of a fault).
    private static final double TARGET_MOVED_DEG = 20;

    private final CachedCRServo servo;
    private final AnalogInput encoder;
    private final VoltageCache voltage;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime fullPowerTimer = new ElapsedTime();

    private double targetDeg = 0;
    private double lastTargetDeg = 0;
    private double trimDeg = 0;
    private double angleDeg = 0;
    private double errorDeg = 0;
    private double lastErrorDeg = 0;
    private boolean closeGains = false;
    private double power = 0;
    private Double manualPower = null;
    private boolean wasFullPower = false;
    private double errorAtFullPowerStart = 0;
    private double targetMovedDeg = 0;
    private boolean watchdogTripped = false;

    public Turret(HardwareMap hardwareMap, VoltageCache voltage) {
        this.voltage = voltage;
        CRServo crServo = hardwareMap.get(CRServo.class, HardwareNames.TURRET_SERVO);
        servo = new CachedCRServo(crServo);
        encoder = hardwareMap.get(AnalogInput.class, HardwareNames.TURRET_ENCODER);
        angleDeg = readAngle();
        targetDeg = angleDeg;
        lastTargetDeg = angleDeg;
    }

    public void setTargetDeg(double degrees) {
        targetDeg = degrees;
    }

    public void addTrim(double degrees) {
        trimDeg += degrees;
    }

    public void resetTrim() {
        trimDeg = 0;
    }

    /** For tuning only: drive the servo directly. Pass null to go back to aiming. */
    public void setManualPower(Double power) {
        manualPower = power;
    }

    public double angleDeg() {
        return angleDeg;
    }

    public double errorDeg() {
        return errorDeg;
    }

    /** False once the watchdog has tripped: a turret that is switched off isn't aimed. */
    public boolean onTarget() {
        return !watchdogTripped && Math.abs(errorDeg) < ON_TARGET_DEG;
    }

    /** True once the turret was cut off for running at full power without closing the error. Stays true until the OpMode restarts. */
    public boolean watchdogTripped() {
        return watchdogTripped;
    }

    public double encoderVolts() {
        return encoder.getVoltage();
    }

    private double readAngle() {
        double raw = encoder.getVoltage() / Math.max(ENCODER_FULL_TURN_VOLTS, MIN_DIVISOR) * 360;
        if (ENCODER_REVERSED) raw = -raw;
        return Angles.wrapDegrees(raw - ENCODER_ZERO_DEG);
    }

    public void update() {
        double dt = Math.max(loopTimer.seconds(), 1e-3);
        loopTimer.reset();

        angleDeg = readAngle();
        double aimDeg = targetDeg + trimDeg;
        errorDeg = Angles.wrapDegrees(aimDeg - angleDeg);
        double targetStep = Angles.wrapDegrees(targetDeg - lastTargetDeg);
        double targetRate = targetStep / dt;
        double errorRate = (errorDeg - lastErrorDeg) / dt;
        lastTargetDeg = targetDeg;
        lastErrorDeg = errorDeg;

        double absError = Math.abs(errorDeg);
        if (absError < CLOSE_ZONE_DEG - CLOSE_ZONE_BAND_DEG) closeGains = true;
        if (absError > CLOSE_ZONE_DEG + CLOSE_ZONE_BAND_DEG) closeGains = false;
        double kP = closeGains ? CLOSE_KP : FAR_KP;
        double kD = closeGains ? CLOSE_KD : FAR_KD;

        double staticPush = Math.signum(errorDeg) * KS * Math.min(absError / Math.max(KS_RAMP_DEG, MIN_DIVISOR), 1);
        double raw = kP * errorDeg + kD * errorRate + staticPush + KV * targetRate;

        if (manualPower != null) {
            // Tuning: the person on the stick is the safety, so the watchdog neither counts nor blocks this.
            power = manualPower;
            wasFullPower = false;
        } else if (watchdogTripped) {
            power = 0;
        } else {
            power = Range.clip(raw * voltage.compensation(), -MAX_POWER, MAX_POWER);
            // Range.clip lets NaN through (NaN target, bad Panels value). Never send that to the servo.
            if (Double.isNaN(power)) power = 0;
            checkWatchdog(absError, targetStep);
            if (watchdogTripped) power = 0;
        }
        servo.setPower(SERVO_REVERSED ? -power : power);
    }

    /**
     * Trips if the turret has been at full power for WATCHDOG_MS and the error is not
     * smaller than when full power started. That catches both runaways:
     *   - encoder unplugged or stuck: the angle never changes, so neither does the error.
     *   - SERVO_REVERSED and ENCODER_REVERSED disagreeing: positive feedback, the error
     *     grows to about 180 and stays there.
     * It looks at the size of the error, not at whether the angle moved, because the
     * angle wraps and a runaway turret moves a lot.
     *
     * A normal big move (D-pad 0 to 180) can't trip it. A healthy turret at full power
     * closes tens of degrees every fraction of a second, so the error is much smaller
     * than at the start long before WATCHDOG_MS. Full power also ends once the error
     * gets small. And every time a window passes with progress, a new window starts from
     * the current error, so even a slow move is fine as long as it keeps closing in.
     *
     * It never trips on a window where the target moved more than TARGET_MOVED_DEG (the
     * robot is spinning faster than the turret can follow). The faults we want happen with
     * the target still. We add up the wrapped steps, so a spin of a full turn still counts.
     */
    private void checkWatchdog(double absError, double targetStep) {
        // power != 0 matters if MAX_POWER is set to 0: no power at all isn't "full power".
        boolean fullPower = power != 0 && Math.abs(power) >= NEAR_MAX_POWER * MAX_POWER;
        if (!fullPower) {
            wasFullPower = false;
            return;
        }
        if (!wasFullPower) {
            wasFullPower = true;
            startFullPowerWindow(absError);
        } else {
            targetMovedDeg += targetStep;
            if (fullPowerTimer.milliseconds() > WATCHDOG_MS) {
                boolean closedIn = absError <= errorAtFullPowerStart - MIN_PROGRESS_DEG;
                boolean targetMoved = Math.abs(targetMovedDeg) > TARGET_MOVED_DEG;
                if (closedIn || targetMoved) {
                    startFullPowerWindow(absError);
                } else {
                    watchdogTripped = true;
                }
            }
        }
    }

    private void startFullPowerWindow(double absError) {
        fullPowerTimer.reset();
        errorAtFullPowerStart = absError;
        targetMovedDeg = 0;
    }

    public void stop() {
        power = 0;
        servo.setPower(0);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret angle", "%.1f -> %.1f (trim %+.1f)", angleDeg, targetDeg + trimDeg, trimDeg);
        telemetry.addData("Turret error / power", "%.1f / %.2f %s", errorDeg, power, closeGains ? "close" : "far");
        if (watchdogTripped) {
            telemetry.addData("Turret WATCHDOG", "TRIPPED: full power but the error did not shrink. Turret is off. Check the encoder wiring and the two REVERSED flags, then restart the OpMode.");
        }
    }
}
