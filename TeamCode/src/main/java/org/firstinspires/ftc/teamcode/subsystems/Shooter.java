package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.HardwareNames;
import org.firstinspires.ftc.teamcode.util.CachedMotor;
import org.firstinspires.ftc.teamcode.util.VoltageCache;

/**
 * Two coupled motors on one flywheel, fixed launch angle, so RPM is the only
 * thing that changes how far a ball goes. Speed comes from the left motor's encoder.
 *
 * Normal control: power = kS + kV * target + kP * error, scaled for battery voltage.
 * While a burst is firing, "boost" switches to full power whenever the wheel
 * drops below target, so it recovers between balls as fast as possible.
 */
@Configurable
public class Shooter {
    // TODO(6): The motors are geared together: test each alone at low power first. Both must push the shooting direction.
    public static boolean LEFT_REVERSED = false;
    public static boolean RIGHT_REVERSED = true;
    // TODO(6): RPM must read positive while shooting. The encoder must be on the left motor's port.
    public static boolean ENCODER_REVERSED = false;

    // TODO(6): Raise if low RPMs sit below target.
    public static double KS = 0.02;
    // TODO(6): Shooter Test with KS and KP at 0: raise until the wheel settles on target at 2000, 3500 and 5000 RPM.
    /** Power per RPM. Start near 1 / free speed (6000). */
    public static double KV = 1.0 / 6000;
    // TODO(6): Raise until it recovers fast without oscillating.
    public static double KP = 0.0005;

    // TODO(6): Largest RPM error that still scores at mid range.
    /** The shot is allowed when the wheel is within this many RPM of target. */
    public static double RPM_TOLERANCE = 75;
    // TODO(6): Fire 4 balls: quick recovery between balls, no big overshoot.
    /** In boost, full power whenever the wheel is more than this far below target. */
    public static double BOOST_BELOW_RPM = 50;

    private static final double TICKS_PER_REV = 28;

    private final CachedMotor left, right;
    private final DcMotorEx encoderMotor;
    private final VoltageCache voltage;

    private double targetRpm = 0;
    private double trimRpm = 0;
    private boolean boost = false;
    private double rpm = 0;
    private double power = 0;

    public Shooter(HardwareMap hardwareMap, VoltageCache voltage) {
        this.voltage = voltage;
        DcMotorEx leftMotor = motor(hardwareMap, HardwareNames.SHOOTER_LEFT, LEFT_REVERSED);
        DcMotorEx rightMotor = motor(hardwareMap, HardwareNames.SHOOTER_RIGHT, RIGHT_REVERSED);
        left = new CachedMotor(leftMotor);
        right = new CachedMotor(rightMotor);
        encoderMotor = leftMotor;
    }

    private static DcMotorEx motor(HardwareMap hardwareMap, String name, boolean reversed) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    /** 0 stops the wheel. Trim is added on top of any non-zero target. */
    public void setTargetRpm(double rpm) {
        targetRpm = rpm;
    }

    public void setBoost(boolean boost) {
        this.boost = boost;
    }

    public void addTrim(double rpm) {
        trimRpm += rpm;
    }

    public void resetTrim() {
        trimRpm = 0;
    }

    public double effectiveTargetRpm() {
        return targetRpm > 0 ? targetRpm + trimRpm : 0;
    }

    public double rpm() {
        return rpm;
    }

    public boolean atSpeed() {
        double target = effectiveTargetRpm();
        return target > 0 && Math.abs(rpm - target) < RPM_TOLERANCE;
    }

    public void update() {
        double ticksPerSecond = encoderMotor.getVelocity();
        rpm = (ENCODER_REVERSED ? -ticksPerSecond : ticksPerSecond) * 60 / TICKS_PER_REV;

        double target = effectiveTargetRpm();
        if (target <= 0) {
            power = 0;
        } else if (boost && rpm < target - BOOST_BELOW_RPM) {
            power = 1;
        } else {
            double raw = KS + KV * target + KP * (target - rpm);
            power = Range.clip(raw * voltage.compensation(), 0, 1);
        }
        left.setPower(power);
        right.setPower(power);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter RPM", "%.0f / %.0f (trim %+.0f)", rpm, effectiveTargetRpm(), trimRpm);
        telemetry.addData("Shooter power", "%.2f%s", power, boost ? " boost" : "");
    }
}
