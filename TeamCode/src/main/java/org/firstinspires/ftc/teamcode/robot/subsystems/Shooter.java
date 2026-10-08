package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.hardware.CachedMotor;
import org.firstinspires.ftc.teamcode.robot.hardware.HardwareNames;
import org.firstinspires.ftc.teamcode.robot.hardware.VoltageCache;

/**
 * Two coupled motors on one flywheel, fixed launch angle, so RPM is the only
 * thing that changes how far a ball goes. Speed comes from the left motor's encoder.
 *
 * Normal control: power = kS + kV * target + kP * error, scaled for battery voltage.
 * While a burst is firing, "boost" switches to full power whenever the wheel
 * drops below target, so it recovers between balls as fast as possible.
 *
 * Call readSensors() at the start of each loop (after the bulk-read clear) and
 * update() at the end. That way atSpeed() and rpm() are this loop's values when
 * the loop decides whether to fire, not last loop's.
 */
@Configurable
public class Shooter {
    // Both motors stay FORWARD in the SDK. The two REVERSED flags flip the sign when power is
    // written, so they take effect live. (The SDK's setDirection would also flip the encoder
    // reading, which is why it isn't used.)
    // TODO(6): The motors are geared together: test each alone at low power first (Shooter Test, hold Square or Circle). Both must push the shooting direction.
    public static boolean LEFT_REVERSED = false;
    public static boolean RIGHT_REVERSED = true;
    // TODO(6): RPM must read positive while shooting. The encoder must be on the left motor's port. This flag alone sets the RPM sign, LEFT_REVERSED does not change it.
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

    // Runaway watchdog. With the encoder unplugged (RPM reads 0) or ENCODER_REVERSED wrong
    // (RPM reads negative), the controller thinks the wheel is stuck and holds full power forever.
    // If the power stays above WATCHDOG_POWER for WATCHDOG_MS while the signed RPM is below
    // WATCHDOG_MIN_RPM, the shooter turns off until the OpMode restarts. The RPM is signed on
    // purpose: a wrong sign reads large negative RPM, which an absolute value would miss.
    // A normal spin-up passes 100 RPM well inside 500 ms, so it never trips.
    public static double WATCHDOG_POWER = 0.5;
    // TODO(6): Only if the watchdog trips on a normal spin-up: raise this, or lower WATCHDOG_MIN_RPM.
    public static double WATCHDOG_MS = 500;
    public static double WATCHDOG_MIN_RPM = 100;

    private static final double TICKS_PER_REV = 28;
    // Finer than the 0.01 default. KV is 1/6000, so 0.01 power is about 60 RPM of feedforward
    // and small KV edits in Panels would look dead.
    private static final double MIN_POWER_CHANGE = 0.002;

    private final CachedMotor left, right;
    private final DcMotorEx encoderMotor;
    private final VoltageCache voltage;
    private final ElapsedTime runawayFor = new ElapsedTime();

    private double targetRpm = 0;
    private double trimRpm = 0;
    private boolean boost = false;
    private double rpm = 0;
    private double power = 0;
    private boolean openLoop = false;
    private double openLeftPower = 0;
    private double openRightPower = 0;
    private boolean wasRunaway = false;
    private boolean watchdogTripped = false;

    public Shooter(HardwareMap hardwareMap, VoltageCache voltage) {
        this.voltage = voltage;
        DcMotorEx leftMotor = motor(hardwareMap, HardwareNames.SHOOTER_LEFT);
        DcMotorEx rightMotor = motor(hardwareMap, HardwareNames.SHOOTER_RIGHT);
        left = new CachedMotor(leftMotor, MIN_POWER_CHANGE);
        right = new CachedMotor(rightMotor, MIN_POWER_CHANGE);
        encoderMotor = leftMotor;
    }

    private static DcMotorEx motor(HardwareMap hardwareMap, String name) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        // TODO(6): After a trim-down the wheel coasts (FLOAT) and atSpeed() blocks shots until it slows. Time a 100 to 200 RPM coast-down; if it blocks shots, tell the programmers to add braking or a small reverse power.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    /** 0 stops the wheel. Trim is added on top of any non-zero target. Also ends an open-loop test. */
    public void setTargetRpm(double rpm) {
        openLoop = false;
        targetRpm = rpm;
    }

    /**
     * Test only: drives each motor directly, -1 to 1, with no closed loop. Positive pushes the
     * shooting direction once the REVERSED flags are right. Stays in effect until setTargetRpm is called.
     */
    public void setOpenLoopTest(double leftPower, double rightPower) {
        openLoop = true;
        openLeftPower = Range.clip(leftPower, -1, 1);
        openRightPower = Range.clip(rightPower, -1, 1);
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

    /** True once the runaway watchdog has shut the shooter off. It stays off until the OpMode restarts. */
    public boolean watchdogTripped() {
        return watchdogTripped;
    }

    /** Reads the wheel speed. Call it at the start of the loop, after the bulk-read clear, before anything uses rpm() or atSpeed(). */
    public void readSensors() {
        double ticksPerSecond = encoderMotor.getVelocity();
        rpm = (ENCODER_REVERSED ? -ticksPerSecond : ticksPerSecond) * 60 / TICKS_PER_REV;
    }

    /** Works out the motor power from the rpm that readSensors() read, and writes it. Call it at the end of the loop. */
    public void update() {
        double leftPower;
        double rightPower;
        if (openLoop) {
            leftPower = openLeftPower;
            rightPower = openRightPower;
            wasRunaway = false;
        } else {
            double target = effectiveTargetRpm();
            if (target <= 0) {
                power = 0;
            } else if (boost && rpm < target - BOOST_BELOW_RPM) {
                power = 1;
            } else {
                double raw = KS + KV * target + KP * (target - rpm);
                power = Range.clip(raw * voltage.compensation(), 0, 1);
            }
            checkWatchdog();
            leftPower = power;
            rightPower = power;
        }

        if (watchdogTripped) {
            power = 0;
            leftPower = 0;
            rightPower = 0;
        }
        left.setPower(LEFT_REVERSED ? -leftPower : leftPower);
        right.setPower(RIGHT_REVERSED ? -rightPower : rightPower);
    }

    /** Trips when the power stays high but the wheel does not turn forward. */
    private void checkWatchdog() {
        boolean runaway = power > WATCHDOG_POWER && rpm < WATCHDOG_MIN_RPM;
        // Start the clock when it begins, not at construction, so a slow init can't trip it.
        if (runaway && !wasRunaway) runawayFor.reset();
        wasRunaway = runaway;
        if (runaway && runawayFor.milliseconds() > WATCHDOG_MS) watchdogTripped = true;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter RPM", "%.0f / %.0f (trim %+.0f)", rpm, effectiveTargetRpm(), trimRpm);
        if (openLoop) {
            telemetry.addData("Shooter power", "open-loop test, left %.2f right %.2f", openLeftPower, openRightPower);
        } else {
            telemetry.addData("Shooter power", "%.2f%s", power, boost ? " boost" : "");
        }
        if (watchdogTripped) {
            telemetry.addData("SHOOTER OFF", "Power was over %.2f for %.0f ms but RPM stayed under %.0f. "
                    + "Check the encoder cable and ENCODER_REVERSED, then restart the OpMode.",
                    WATCHDOG_POWER, WATCHDOG_MS, WATCHDOG_MIN_RPM);
        }
    }
}
