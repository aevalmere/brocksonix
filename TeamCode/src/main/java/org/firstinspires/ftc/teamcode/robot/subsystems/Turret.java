package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.hardware.CachedCRServo;
import org.firstinspires.ftc.teamcode.robot.hardware.HardwareNames;
import org.firstinspires.ftc.teamcode.robot.hardware.VoltageCache;
import org.firstinspires.ftc.teamcode.util.Angles;

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
 * Safety. Ask watchdogTripped() whether the turret is off until the OpMode restarts.
 *   - Wrong way: it turns against its own push, more than WRONG_WAY_DEG overall. That is
 *     SERVO_REVERSED and ENCODER_REVERSED disagreeing: off within about 1.5 s (about 0.3 s
 *     when the target is near). The encoder sits on the turret and measures against the
 *     chassis, so the robot turning doesn't change it.
 *   - Full power, but the error doesn't shrink for WATCHDOG_MS: off. A backup for anything
 *     else that keeps the error from closing; the wrong-way check sees wrong REVERSED
 *     flags first.
 *   - Stuck: it pushes at KS or more but the angle doesn't move. Friction, a dead encoder
 *     and a held turret all look like that, so it tests: it pushes harder, little by
 *     little, up to TEST_POWER. Friction gives way and the turret moves, so all is well.
 *     If it still doesn't move, the power goes off for REST_MS and it tries again.
 *     After FAILED_TESTS_TO_LATCH failed tests in a row it stays off. So a turret held
 *     for a few seconds comes back, and a dead encoder ends up off in about 4 to 6 s,
 *     at any error (this check sees it before the full-power one would).
 *   - Under KS: a push under KS can't start a turret that is standing still, but it can
 *     keep one creeping that was already moving (moving friction is lower). So once the
 *     angle has stood still for STUCK_WATCH_MS, a push under KS is cut to 0. A working
 *     turret at rest doesn't notice; an encoder that froze mid-move stops the creep.
 *   - Not caught: an encoder that freezes close to the target, under the KS power. The
 *     turret creeps for up to STUCK_WATCH_MS, then stops, and onTarget() believes the
 *     frozen reading. A frozen reading and a turret at rest look the same.
 *
 * Call readSensors() at the start of each loop (after the bulk-read clear) and
 * update() at the end. That way angleDeg(), errorDeg() and onTarget() are this
 * loop's values when the loop decides whether to fire, not last loop's.
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

    // TODO(5): Manual mode (KS isn't used there): push the stick slowly, note the power where the turret starts to move, set KS a bit below it.
    public static double KS = 0.05;
    // TODO(5): Aiming mode: if it chatters near the target, adjust this. Higher = KS fades in over a wider zone.
    public static double KS_RAMP_DEG = 5;
    // TODO(11): Main TeleOp: spin the robot in place. Raise until the turret stops lagging behind the cell.
    /** Power per degree/second of target motion. */
    public static double KV = 0.0;
    // TODO(5): Lower if the gearing or wiring complains.
    public static double MAX_POWER = 0.8;

    // TODO(5): Largest aim error that still scores. Find it once the shooter works.
    public static double ON_TARGET_DEG = 3;

    // TODO(5): Aiming mode, D-pad 0 to 180: if a normal big move trips the watchdog, raise this time.
    /** The full-power check trips if the turret stays at full power this long and the error hasn't shrunk. */
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
    // Stuck check. It watches while the power is at least KS (and at least STUCK_MIN_POWER, in case
    // KS is still 0): the turret should be moving then. Pushing for STUCK_WATCH_MS without moving starts a test.
    private static final double STUCK_MIN_POWER = 0.05;
    private static final double STUCK_WATCH_MS = 500;
    // The angle has to change more than this to count as moving. It is well above the encoder's noise.
    private static final double STUCK_ANGLE_DEG = 3;
    // Under KS, a push is cut to 0 once the angle hasn't moved more than this for STUCK_WATCH_MS.
    // Smaller than STUCK_ANGLE_DEG, so a turret that is following a slowly turning robot keeps its push.
    private static final double STILL_ANGLE_DEG = 1;
    // The test push grows this much per second, up to TEST_POWER. Slow growth means friction gives way
    // just above the power it needs, so the turret doesn't jump.
    private static final double TEST_RAMP_PER_SEC = 1.0;
    // Every working turret turns at TEST_POWER: several times a normal KS. A stiff turret with a big
    // KS is tested at TEST_KS_TIMES * KS instead, if that is more.
    private static final double TEST_POWER = 0.4;
    private static final double TEST_KS_TIMES = 2;
    // Not moving after this long at TEST_POWER fails the test.
    private static final double TEST_HOLD_MS = 200;
    // After a failed test the power is off this long, then it watches again.
    private static final double REST_MS = 1000;
    private static final int FAILED_TESTS_TO_LATCH = 3;
    // Wrong-way check: turning against the push adds up, turning with it takes away (never below 0).
    // Over this many degrees trips. A working turret only turns against its push while it brakes or
    // something shoves it, a few tens of degrees at most; wrong REVERSED flags pass 90 within about 1.5 s
    // (about 0.3 s when the target is near).
    private static final double WRONG_WAY_DEG = 90;
    // Faster than the turret can turn. A reading that jumps more than this in one loop is a bad reading
    // (the encoder's noisy top end), so that loop doesn't count. Then one bad reading can't trip it.
    private static final double TOO_FAST_DEG_PER_SEC = 2000;

    private enum StuckStep { WATCH, TEST, REST }

    private final CachedCRServo servo;
    private final AnalogInput encoder;
    private final VoltageCache voltage;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime fullPowerTimer = new ElapsedTime();
    private final ElapsedTime stuckTimer = new ElapsedTime();
    private final ElapsedTime stillTimer = new ElapsedTime();

    private double targetDeg = 0;
    private double lastTargetDeg = 0;
    private double trimDeg = 0;
    private double angleDeg = 0;
    private double lastErrorDeg = 0;
    private boolean closeGains = false;
    private double power = 0;
    private Double manualPower = null;
    private boolean wasFullPower = false;
    private double errorAtFullPowerStart = 0;
    private double targetMovedDeg = 0;
    private StuckStep stuckStep = StuckStep.WATCH;
    private boolean watching = false;
    private double pushSign = 0;
    private double angleAtPushStart = 0;
    private double testStartPower = 0;
    private int failedTests = 0;
    private boolean watchingStill = false;
    private double angleAtStillStart = 0;
    private double angleLastLoop = 0;
    private double powerLastLoop = 0;
    private double wrongWayDeg = 0;
    private boolean watchdogTripped = false;
    /** Which check tripped the watchdog, for telemetry. */
    private String tripReason = "";

    public Turret(HardwareMap hardwareMap, VoltageCache voltage) {
        this.voltage = voltage;
        CRServo crServo = hardwareMap.get(CRServo.class, HardwareNames.TURRET_SERVO);
        servo = new CachedCRServo(crServo);
        encoder = hardwareMap.get(AnalogInput.class, HardwareNames.TURRET_ENCODER);
        angleDeg = readAngle();
        targetDeg = angleDeg;
        lastTargetDeg = angleDeg;
        angleLastLoop = angleDeg;
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

    /**
     * Shortest way from the angle readSensors() read to the aimed angle (target plus trim).
     * It is worked out on each call instead of stored, so it is never a loop old: the
     * target can change after readSensors() and onTarget() still sees it.
     */
    public double errorDeg() {
        return Angles.wrapDegrees(targetDeg + trimDeg - angleDeg);
    }

    /** False while the turret is off (watchdog), or being tested or rested by the stuck check: it isn't aimed then. */
    public boolean onTarget() {
        return !watchdogTripped && stuckStep == StuckStep.WATCH && Math.abs(errorDeg()) < ON_TARGET_DEG;
    }

    /** True once the watchdog cut the turret off (see the class comment). Stays true until the OpMode restarts. */
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

    /** Reads the turret angle. Call it at the start of the loop, after the bulk-read clear, before anything uses angleDeg() or onTarget(). */
    public void readSensors() {
        angleDeg = readAngle();
    }

    /** Works out the servo power from the angle that readSensors() read, and writes it. Call it at the end of the loop. */
    public void update() {
        double dt = Math.max(loopTimer.seconds(), 1e-3);
        loopTimer.reset();

        double errorDeg = errorDeg();
        double targetStep = Angles.wrapDegrees(targetDeg - lastTargetDeg);
        double targetRate = targetStep / dt;
        double errorRate = (errorDeg - lastErrorDeg) / dt;
        lastTargetDeg = targetDeg;
        lastErrorDeg = errorDeg;

        // How far the turret turned since last loop, for the wrong-way check.
        double turnedDeg = Angles.wrapDegrees(angleDeg - angleLastLoop);
        angleLastLoop = angleDeg;

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
            stuckStep = StuckStep.WATCH;
            watching = false;
            watchingStill = false;
            failedTests = 0;
            wrongWayDeg = 0;
        } else if (watchdogTripped) {
            power = 0;
        } else {
            // Last loop's push moved the turret between last loop's reading and this one.
            checkWrongWay(turnedDeg, dt);
            power = Range.clip(raw * voltage.compensation(), -MAX_POWER, MAX_POWER);
            // Range.clip lets NaN through (NaN target, bad Panels value). Never send that to the servo.
            if (Double.isNaN(power)) power = 0;
            // The stuck check may push harder or rest, so it goes first: the full-power check
            // must look at the power that is really sent.
            power = checkStuck(power);
            checkWatchdog(absError, targetStep);
            if (watchdogTripped) power = 0;
        }
        powerLastLoop = power;
        servo.setPower(SERVO_REVERSED ? -power : power);
    }

    /**
     * Trips if the turret keeps turning against its own push: SERVO_REVERSED and
     * ENCODER_REVERSED disagree, so every push turns it the wrong way and it runs off.
     * Turning against the push adds to wrongWayDeg, turning with it takes away, and it
     * never goes below 0. Only loops with a real push count (at least the stuck line).
     *
     * A working turret turns with its push, so the count stays near 0. It only turns
     * against it for a moment: braking, a shove from another robot, or noise (and noise
     * cancels out from one loop to the next). A loop where the reading jumps faster than
     * TOO_FAST_DEG_PER_SEC is a bad reading and doesn't count. Holding still, the stuck
     * check's test, the KS ramp near the target (under the line) and manual mode add
     * nothing. The encoder measures the turret against the chassis, so the robot turning
     * doesn't count either.
     */
    private void checkWrongWay(double turnedDeg, double dt) {
        double line = Math.max(KS, STUCK_MIN_POWER) * voltage.compensation();
        if (Math.abs(powerLastLoop) < line) return;
        // A NaN here would stick in wrongWayDeg for good.
        if (Double.isNaN(turnedDeg) || Math.abs(turnedDeg) > TOO_FAST_DEG_PER_SEC * dt) return;
        double againstPush = -Math.signum(powerLastLoop) * turnedDeg;
        wrongWayDeg = Math.max(0, wrongWayDeg + againstPush);
        if (wrongWayDeg > WRONG_WAY_DEG) {
            trip(String.format("turned %.0f deg against its push: SERVO_REVERSED and ENCODER_REVERSED disagree (or the encoder reads garbage)", wrongWayDeg));
        }
    }

    /**
     * Trips if the turret has been at full power for WATCHDOG_MS and the error is not
     * smaller than when full power started. That catches SERVO_REVERSED and
     * ENCODER_REVERSED disagreeing: positive feedback, the error grows to about 180 and
     * stays there. (An unplugged or stuck encoder would trip it too, but the stuck check
     * sees that first, at 500 ms, and its rest restarts this window. So a dead encoder
     * ends up off through the stuck check instead, in about 4 to 6 s.)
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
                    trip("full power but the error did not shrink");
                }
            }
        }
    }

    private void startFullPowerWindow(double absError) {
        fullPowerTimer.reset();
        errorAtFullPowerStart = absError;
        targetMovedDeg = 0;
    }

    /**
     * The stuck check (see the class comment). Takes the power the controller wants and
     * returns the power to send. Three steps, like FireControl:
     *
     * WATCH: if the turret pushes the same way, at KS or more, for STUCK_WATCH_MS and the
     *   angle doesn't move more than STUCK_ANGLE_DEG, go to TEST. Pushing at KS should turn
     *   it, since KS is just under the power that does.
     * TEST: push harder, TEST_RAMP_PER_SEC more each second, up to TEST_POWER. If the angle
     *   moves, it was only friction: back to WATCH (the push even helped it to the target).
     *   If it still hasn't moved after TEST_HOLD_MS at TEST_POWER, the reading can't be
     *   trusted (encoder frozen or unplugged) or the turret is held: the test fails.
     * REST: power off for REST_MS, then WATCH again. FAILED_TESTS_TO_LATCH failed tests in
     *   a row turn the turret off until the OpMode restarts. Any real movement resets the count.
     *
     * The test only runs while the controller still wants that push: if the target moves
     * so the push drops under the line or turns around, it goes back to WATCH.
     *
     * Under the line, WATCH also cuts the push to 0 once the angle has stood still
     * (cutUnderKs below).
     */
    private double checkStuck(double wanted) {
        // Like KS itself, both of these are scaled for battery voltage.
        double line = Math.max(KS, STUCK_MIN_POWER) * voltage.compensation();
        double testPower = Math.min(Math.max(TEST_POWER, TEST_KS_TIMES * KS) * voltage.compensation(), MAX_POWER);
        boolean pushing = Math.abs(wanted) >= line;
        boolean sameWay = Math.signum(wanted) == pushSign;
        boolean moved = Math.abs(Angles.wrapDegrees(angleDeg - angleAtPushStart)) > STUCK_ANGLE_DEG;

        switch (stuckStep) {
            case WATCH:
                if (!pushing) {
                    watching = false;
                    return cutUnderKs(wanted);
                }
                watchingStill = false;
                if (!watching || !sameWay || moved) {
                    if (watching && moved) failedTests = 0;
                    startWatching(wanted);
                } else if (stuckTimer.milliseconds() >= STUCK_WATCH_MS) {
                    stuckStep = StuckStep.TEST;
                    testStartPower = Math.abs(wanted);
                    stuckTimer.reset();
                }
                return wanted;

            case TEST:
                if (!pushing || !sameWay) {
                    stuckStep = StuckStep.WATCH;
                    watching = false;
                    return wanted;
                }
                if (moved) {
                    failedTests = 0;
                    stuckStep = StuckStep.WATCH;
                    startWatching(wanted);
                    return wanted;
                }
                double seconds = stuckTimer.seconds();
                double push = Math.min(testStartPower + TEST_RAMP_PER_SEC * seconds, testPower);
                double secondsToTestPower = Math.max(testPower - testStartPower, 0) / TEST_RAMP_PER_SEC;
                if (seconds >= secondsToTestPower + TEST_HOLD_MS / 1000) {
                    failedTests++;
                    if (failedTests >= FAILED_TESTS_TO_LATCH) {
                        trip("pushed up to TEST_POWER " + failedTests + " times and the angle never moved (encoder frozen or unplugged, or turret jammed)");
                    }
                    stuckStep = StuckStep.REST;
                    stuckTimer.reset();
                    return 0;
                }
                return pushSign * Math.max(Math.abs(wanted), push);

            default: // REST
                if (stuckTimer.milliseconds() >= REST_MS) {
                    stuckStep = StuckStep.WATCH;
                    watching = false;
                }
                return 0;
        }
    }

    /**
     * For a push under the stuck line (under KS). KS is just under the power that starts the
     * turret from standing still, so a turret at rest stays at rest with this push or without
     * it. A turret that is already moving needs less (moving friction is lower), so if the
     * encoder froze mid-move, this push would keep it creeping and nothing would see it. So
     * once the angle hasn't moved more than STILL_ANGLE_DEG for STUCK_WATCH_MS, send 0.
     */
    private double cutUnderKs(double wanted) {
        boolean moved = Math.abs(Angles.wrapDegrees(angleDeg - angleAtStillStart)) > STILL_ANGLE_DEG;
        if (!watchingStill || moved) {
            watchingStill = true;
            angleAtStillStart = angleDeg;
            stillTimer.reset();
        }
        return stillTimer.milliseconds() >= STUCK_WATCH_MS ? 0 : wanted;
    }

    private void startWatching(double wanted) {
        watching = true;
        pushSign = Math.signum(wanted);
        angleAtPushStart = angleDeg;
        stuckTimer.reset();
    }

    private void trip(String reason) {
        watchdogTripped = true;
        tripReason = reason;
    }

    public void stop() {
        power = 0;
        servo.setPower(0);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret angle", "%.1f -> %.1f (trim %+.1f)", angleDeg, targetDeg + trimDeg, trimDeg);
        telemetry.addData("Turret error / power", "%.1f / %.2f %s", errorDeg(), power, closeGains ? "close" : "far");
        if (watchdogTripped) {
            telemetry.addData("Turret WATCHDOG", "TRIPPED: " + tripReason + ". Turret is off. Check the encoder wiring and the two REVERSED flags, then restart the OpMode.");
        } else if (stuckStep != StuckStep.WATCH || failedTests > 0) {
            telemetry.addData("Turret stuck check", "%s, failed tests %d of %d", stuckStep, failedTests, FAILED_TESTS_TO_LATCH);
        }
    }
}
