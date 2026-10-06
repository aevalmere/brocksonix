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

    // TODO(5): Aiming mode, D-pad 0 to 180: fast with a small overshoot.
    public static double FAR_KP = 0.01;
    public static double FAR_KD = 0.0005;
    // TODO(5): Aiming mode, bumper nudges: settles without buzzing.
    public static double CLOSE_KP = 0.006;
    public static double CLOSE_KD = 0.0003;
    /** Use CLOSE gains under this error; the band stops it flickering between sets. */
    public static double CLOSE_ZONE_DEG = 15;
    public static double CLOSE_ZONE_BAND_DEG = 2;

    // TODO(5): Manual mode: raise until the turret just starts to move, then use a bit less.
    public static double KS = 0.05;
    public static double KS_RAMP_DEG = 5;
    // TODO(5): Main TeleOp: spin the robot in place. Raise until the turret stops lagging behind the cell.
    /** Power per degree/second of target motion. */
    public static double KV = 0.0;
    public static double MAX_POWER = 0.8;

    // TODO(5): Largest aim error that still scores. Find it once the shooter works.
    public static double ON_TARGET_DEG = 3;

    private final CachedCRServo servo;
    private final AnalogInput encoder;
    private final VoltageCache voltage;
    private final ElapsedTime loopTimer = new ElapsedTime();

    private double targetDeg = 0;
    private double lastTargetDeg = 0;
    private double trimDeg = 0;
    private double angleDeg = 0;
    private double errorDeg = 0;
    private double lastErrorDeg = 0;
    private boolean closeGains = false;
    private double power = 0;
    private Double manualPower = null;

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

    public boolean onTarget() {
        return Math.abs(errorDeg) < ON_TARGET_DEG;
    }

    public double encoderVolts() {
        return encoder.getVoltage();
    }

    private double readAngle() {
        double raw = encoder.getVoltage() / ENCODER_FULL_TURN_VOLTS * 360;
        if (ENCODER_REVERSED) raw = -raw;
        return Angles.wrapDegrees(raw - ENCODER_ZERO_DEG);
    }

    public void update() {
        double dt = Math.max(loopTimer.seconds(), 1e-3);
        loopTimer.reset();

        angleDeg = readAngle();
        double aimDeg = targetDeg + trimDeg;
        errorDeg = Angles.wrapDegrees(aimDeg - angleDeg);
        double targetRate = Angles.wrapDegrees(targetDeg - lastTargetDeg) / dt;
        double errorRate = (errorDeg - lastErrorDeg) / dt;
        lastTargetDeg = targetDeg;
        lastErrorDeg = errorDeg;

        double absError = Math.abs(errorDeg);
        if (absError < CLOSE_ZONE_DEG - CLOSE_ZONE_BAND_DEG) closeGains = true;
        if (absError > CLOSE_ZONE_DEG + CLOSE_ZONE_BAND_DEG) closeGains = false;
        double kP = closeGains ? CLOSE_KP : FAR_KP;
        double kD = closeGains ? CLOSE_KD : FAR_KD;

        double staticPush = Math.signum(errorDeg) * KS * Math.min(absError / KS_RAMP_DEG, 1);
        double raw = kP * errorDeg + kD * errorRate + staticPush + KV * targetRate;

        if (manualPower != null) {
            power = manualPower;
        } else {
            power = Range.clip(raw * voltage.compensation(), -MAX_POWER, MAX_POWER);
        }
        servo.setPower(SERVO_REVERSED ? -power : power);
    }

    public void stop() {
        power = 0;
        servo.setPower(0);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret angle", "%.1f -> %.1f (trim %+.1f)", angleDeg, targetDeg + trimDeg, trimDeg);
        telemetry.addData("Turret error / power", "%.1f / %.2f %s", errorDeg, power, closeGains ? "close" : "far");
    }
}
