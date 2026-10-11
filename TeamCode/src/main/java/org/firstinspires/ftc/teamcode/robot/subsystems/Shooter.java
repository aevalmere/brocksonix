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
import org.firstinspires.ftc.teamcode.util.PIDF;

/**
 * Two coupled motors on one flywheel. Speed comes from the left motor's encoder.
 * With the static hood, RPM is the only thing that changes how far a ball goes.
 * With the coax hood, the hood angle is solved from averageRpm(), the speed the
 * wheel really has, so the wheel doesn't have to be exactly at target.
 *
 * Normal control: power = kS + kV * target + kP * error + kA * how fast the target
 * changes, scaled for battery voltage. The kA part pushes the wheel along while the
 * target changes as the robot drives closer or farther, so it doesn't lag behind. It
 * is 0 when the target holds still, so it never biases a shot. It is GNCE Ditto's
 * FLYWHEEL_ACCEL_KFF: https://github.com/GNCE/DECODEv2-Ditto/blob/c78b5268930d55c6d5a59c23922d9a8baf9f96e6/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/subsystems/Shooter.java
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
    // TODO(11): Main TeleOp: drive straight toward and away from the HIVE while watching Shooter RPM. Raise from 0 until the wheel keeps up with the target without overshooting it.
    /** Power per RPM-per-second that the target changes. */
    public static double KA = 0;
    // TODO(11): Raise (smoother) if the kA part jumps around; lower if it reacts too late.
    /** Smoothing for the target-change estimate: 0 = none, closer to 1 = smoother but later (same as the turret's filters). */
    public static double TARGET_RATE_FILTER = 0.7;
    // TODO(11): With KA on, drive full speed straight at the HIVE and watch "target changing" in telemetry. Set this a little above the biggest number you see.
    /** Limit on the target-change estimate, in RPM per second. */
    public static double MAX_TARGET_RATE = 8000;

    // TODO(16): Coax hood: if the hood shakes with the wheel's speed reading, raise it. Each sample is one loop; more = steadier but later.
    /** Loops averaged for averageRpm(), the speed the coax hood solves with. The average runs about half that many loops late. */
    public static int AVERAGE_SAMPLES = 3;

    // TODO(6): Largest RPM error that still scores at mid range. Not under 90: the RPM reading moves in steps of about 43.
    /**
     * The shot is allowed when the wheel is within this many RPM of target. At least 2 steps of
     * the RPM reading (2 x 43 = 86, see TICKS_PER_REV), so one step of reading noise can't flip
     * atSpeed() on and off. FireControl.READY_HOLD_MS then makes it hold before a burst starts.
     */
    public static double RPM_TOLERANCE = 90;
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

    // Encoder counts per turn of a bare goBILDA 6000 RPM motor. The wheel is driven 1:1 (ROBOT.md),
    // so motor RPM is wheel RPM; a belt or gears between them would need their ratio in readSensors().
    // getVelocity() is ticks per second over the hub's last 50 ms, so it moves in steps of
    // 20 ticks/s: 20 * 60 / 28 = about 43 RPM per step. Full speed is 2800 ticks/s, far under
    // 32767, where the hub's 16-bit velocity would wrap around.
    private static final double TICKS_PER_REV = 28;
    // Finer than the 0.01 default. KV is 1/6000, so 0.01 power is about 60 RPM of feedforward
    // and small KV edits in Panels would look dead.
    private static final double MIN_POWER_CHANGE = 0.002;
    // A target change bigger than this in one loop is a jump (spin-up from 0, shoot to pass),
    // not the robot driving, so it doesn't feed the kA estimate. Trim presses are left out too.
    private static final double TARGET_JUMP_RPM = 200;
    // Room for AVERAGE_SAMPLES; Panels values above this are capped.
    private static final int MAX_AVERAGE_SAMPLES = 20;

    private final CachedMotor left, right;
    private final DcMotorEx encoderMotor;
    private final VoltageCache voltage;
    private final ElapsedTime runawayFor = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final double[] recentRpm = new double[MAX_AVERAGE_SAMPLES];
    private int recentCount = 0;
    private int recentNext = 0;

    private double targetRpm = 0;
    private double trimRpm = 0;
    private boolean boost = false;
    private double rpm = 0;
    private double power = 0;
    private double lastTarget = 0;
    private double targetRate = 0;
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
        // TODO(6): Shooter Test, wheel on: D-pad down once or twice and time the 100 to 200 RPM coast-down. Driving toward the HIVE and trimming down both lower the target, and atSpeed() holds off a burst until the wheel gets there. If that takes too long, tell the programmers to add braking or a small reverse power.
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

    /**
     * The average of the last AVERAGE_SAMPLES readings: steadier than rpm(), a loop or so later.
     * The coax hood solves with it, like GNCE Ditto's ShotPlanner.addRpmSample (they averaged 8).
     */
    public double averageRpm() {
        int samples = Math.min(Math.max(AVERAGE_SAMPLES, 1), Math.min(recentCount, MAX_AVERAGE_SAMPLES));
        if (samples == 0) return rpm;
        double sum = 0;
        for (int i = 1; i <= samples; i++) {
            sum += recentRpm[(recentNext - i + MAX_AVERAGE_SAMPLES) % MAX_AVERAGE_SAMPLES];
        }
        return sum / samples;
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
        recentRpm[recentNext] = rpm;
        recentNext = (recentNext + 1) % MAX_AVERAGE_SAMPLES;
        recentCount = Math.min(recentCount + 1, MAX_AVERAGE_SAMPLES);
    }

    /** Works out the motor power from the rpm that readSensors() read, and writes it. Call it at the end of the loop. */
    public void update() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        updateTargetRate(dt);

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
                double raw = KS + KV * target + KP * (target - rpm) + KA * targetRate;
                power = Range.clip(raw * voltage.compensation(), 0, 1);
                // Range.clip lets NaN through (a bad Panels value). Never send that to the motors.
                if (Double.isNaN(power)) power = 0;
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

    /** How fast the target RPM is changing, smoothed, for the kA part. Jumps and odd loop times count as no change. */
    private void updateTargetRate(double dt) {
        // Without the trim, like the turret: a trim press isn't the robot driving.
        double target = Math.max(targetRpm, 0);
        double step = target - lastTarget;
        lastTarget = target;
        double rawRate = 0;
        if (dt > 0.001 && dt < 0.1 && Math.abs(step) < TARGET_JUMP_RPM) {
            rawRate = Range.clip(step / dt, -MAX_TARGET_RATE, MAX_TARGET_RATE);
        }
        targetRate = PIDF.smooth(targetRate, rawRate, TARGET_RATE_FILTER);
        // A NaN from a bad Panels value would stay in targetRate for good.
        if (Double.isNaN(targetRate)) targetRate = 0;
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
            telemetry.addData("Shooter power", "%.2f%s (target changing %+.0f rpm/s)", power, boost ? " boost" : "", targetRate);
        }
        if (watchdogTripped) {
            telemetry.addData("SHOOTER OFF", "Power was over %.2f for %.0f ms but RPM stayed under %.0f. "
                    + "Check the encoder cable and ENCODER_REVERSED, then restart the OpMode.",
                    WATCHDOG_POWER, WATCHDOG_MS, WATCHDOG_MIN_RPM);
        }
    }
}
