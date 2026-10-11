package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.hardware.CachedServo;
import org.firstinspires.ftc.teamcode.robot.hardware.HardwareNames;

/**
 * Coax hood: a hood that pivots around the flywheel's axle. The ball rolls
 * between the wheel and the hood and leaves where the hood ends, so turning the
 * hood 1 degree turns the launch direction 1 degree. One positional servo turns
 * the hood through gearing.
 *
 * All angles here are launch angles: degrees above horizontal of the direction
 * the ball leaves in (0 = flat, 90 = straight up). Not servo angles.
 *
 * From launch angle to servo position:
 *   position = SERVO_AT_ANCHOR + (angle - ANCHOR_DEG) * GEAR_RATIO / SERVO_RANGE_DEG
 * with minus instead of plus when REVERSED. The anchor is one measured point:
 * SERVO_AT_ANCHOR puts the hood at ANCHOR_DEG. MIN_LAUNCH_DEG and MAX_LAUNCH_DEG
 * are only limits: the angle is clipped to them first, so the hood never drives
 * into its stops, but changing them never moves any angle. The position is then
 * clipped to 0..1. update() works it out from the statics every loop, so Panels
 * edits apply live.
 *
 * A servo doesn't report where it is (Servo.getPosition() only gives back the
 * last position sent), so arrival is a guess: estimatedDeg()
 * moves toward the commanded angle at SPEED_DEG_PER_SEC, and atTarget() is true
 * once the guess gets there.
 *
 * Robot.HOOD picks the hood. COAX aims with this one. STATIC uses the fixed-angle
 * tables, and if this hood is on the robot it holds STATIC_LAUNCH_DEG.
 * GEAR_RATIO comes from the gearing. Coax Hood Test finds the numbers after it,
 * in the order their TODOs number them (First, Second, ...).
 *
 * A mapping that asks for a servo position past 0 to 1 can't be reached: the servo
 * stops at its end, but the estimate still arrives. servoAtEnd() says so, and Robot
 * shows the HOOD_AT_END warning.
 *
 * The angle-to-servo mapping through a gear ratio follows GNCE Ditto's
 * hoodAngleToServoPos (their hood: 300/44 gearing on a 255-degree servo), but
 * pinned at a measured anchor instead of the top of the range:
 * https://github.com/GNCE/DECODEv2-Ditto/blob/c78b5268930d55c6d5a59c23922d9a8baf9f96e6/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/subsystems/Shooter.java
 */
@Configurable
public class CoaxHood {
    // TODO(16): The gearing isn't built yet. Set this when it is: teeth on the hood's gear divided by teeth on the servo's gear.
    /**
     * Servo degrees per hood degree (a hood degree is a launch degree). Meshing teeth
     * move together, so servo degrees x servo teeth = hood degrees x hood teeth, and
     * GEAR_RATIO = hood teeth / servo teeth. Example: a 20-tooth servo gear driving a
     * 60-tooth hood gear gives 60 / 20 = 3, the servo turns 3 degrees per hood degree.
     * A partial (sector) hood gear counts the teeth its full circle would have.
     */
    public static double GEAR_RATIO = 1;
    // TODO(16): First. Coax Hood Test, angle mode, Circle (raw servo): nudge the hood until it just touches its flat stop. Measure the launch angle there (phone level on the hood's exit lip) and put it here.
    /** A launch angle you measured. The flat stop is easy to hold a level against, but any measured angle works. */
    public static double ANCHOR_DEG = 40;
    // TODO(16): First, same spot as ANCHOR_DEG: the raw servo position the test shows there.
    /** Servo position that puts the hood at ANCHOR_DEG. */
    public static double SERVO_AT_ANCHOR = 0.5;
    // TODO(16): Second. Coax Hood Test, angle mode: D-pad up must make the hood steeper. If it gets flatter (or pushes into the flat stop), flip this.
    /** True if a bigger servo position makes the launch angle smaller. */
    public static boolean REVERSED = false;
    // TODO(16): Third. Start from the servo's spec. Then angle mode: command a steep angle short of the steep stop and measure it. Steeper than commanded: raise this. Flatter: lower it (or GEAR_RATIO is off).
    /**
     * How far the servo turns from position 0 to 1, in degrees. It depends on the servo type
     * picked in the robot configuration too: "Servo" sends 600 to 2400 us pulses, "Full Range
     * Servo" 500 to 2500 us, which goBILDA and Axon servos need to reach their spec range.
     * Example: an Axon MINI MK2 turns 0.175 degrees per us, so 350 degrees on Full Range Servo
     * but only 315 on plain Servo.
     */
    public static double SERVO_RANGE_DEG = 300;
    // TODO(16): Fourth. A degree or two steeper than the flat stop's angle (ANCHOR_DEG, if the anchor is at the stop), so the servo never pushes into it.
    /** Flattest launch angle the hood is sent to. Only a limit: changing it moves no angle. */
    public static double MIN_LAUNCH_DEG = 40;
    // TODO(16): Fourth. Raw servo (Circle): move the hood to its steep stop and measure the angle there. Set this a degree or two flatter.
    /** Steepest launch angle the hood is sent to. Only a limit: changing it moves no angle. */
    public static double MAX_LAUNCH_DEG = 75;
    // TODO(16): Fifth, after MIN and MAX. Coax Hood Test, angle mode: Triangle jumps between MIN and MAX. Time it with slow-motion video, (MAX - MIN) / seconds. Round down, so atTarget() is never early.
    /**
     * How fast the HOOD (not the servo) turns, in degrees per second. Only used to guess when it
     * has arrived, so too high fires before the hood gets there. 200 is a placeholder. Spec check:
     * the servo's degrees per second / GEAR_RATIO, and slower under load. At 4.8 V an Axon MINI MK2
     * or goBILDA Speed servo turns 60 degrees in 0.11 s (545 deg/s), a goBILDA Torque servo in
     * 0.25 s (240 deg/s). With a GEAR_RATIO of 3, even 545 is only about 180 at the hood.
     */
    public static double SPEED_DEG_PER_SEC = 200;
    // TODO(16): Main TeleOp, COAX: raise it if bursts wait on "moving" while the hood angle barely changes; lower it if shots fire before the hood has settled.
    /** atTarget() is true when the estimate is this close to the commanded angle. */
    public static double ON_ANGLE_DEG = 0.5;

    // TODO(16): Measure the static hood's launch angle (phone level on its exit lip).
    /** Where the hood holds when Robot.HOOD is STATIC, so the static hood's SHOT_RPM table still fits. */
    public static double STATIC_LAUNCH_DEG = 60;
    // TODO(16): Main TeleOp, COAX: set it near the angle most shots use, so the hood has the least to move when a shot comes up.
    /** Where the hood goes when there's nothing to aim at. */
    public static double IDLE_LAUNCH_DEG = 60;
    // TODO(16): Once passing is on (FireControl.PASS_ENABLED): higher lobs over robots, lower ones get there sooner.
    /**
     * The lob angle passes aim for. The pass RPM is what lands the ball on the pass target at
     * this angle; the hood then solves for the wheel's real RPM, like a shot.
     */
    public static double PASS_LAUNCH_DEG = 60;

    // A slow loop counts as at most this long, so the estimate never runs ahead of the hood.
    private static final double MAX_LOOP_SECONDS = 0.1;
    // Panels can set these to 0. Dividing by that gives Infinity or NaN.
    private static final double MIN_SERVO_RANGE_DEG = 1;
    private static final double MIN_SPEED_DEG_PER_SEC = 1;

    private final CachedServo servo;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime sinceEstimateStart = new ElapsedTime();
    /** False until the first update(), and while a test drives the servo directly. */
    private boolean estimating = false;
    private double targetDeg = IDLE_LAUNCH_DEG;
    private double trimDeg = 0;
    private double estimatedDeg = 0;
    private double position = Double.NaN;
    /** The last update() wanted a position past 0 to 1, so the servo sits at its end instead. */
    private boolean atEnd = false;
    private Double testPosition = null;

    public CoaxHood(HardwareMap hardwareMap) {
        servo = new CachedServo(hardwareMap.get(Servo.class, HardwareNames.HOOD_SERVO));
    }

    /** True if the robot configuration has the hood servo. The robot runs without the coax hood. */
    public static boolean isConfigured(HardwareMap hardwareMap) {
        return hardwareMap.tryGet(Servo.class, HardwareNames.HOOD_SERVO) != null;
    }

    /** Launch angle to aim at. NaN or Infinity (no shot) is ignored, so the hood keeps its last target. */
    public void setTargetDeg(double launchDeg) {
        if (!Double.isFinite(launchDeg)) return;
        // Stored unclipped. commandedDeg() clips, so MIN and MAX edits in Panels apply live.
        targetDeg = launchDeg;
    }

    /** Drivers' correction during a match, added to the target. */
    public void addTrim(double degrees) {
        if (!Double.isFinite(degrees)) return;
        trimDeg += degrees;
    }

    public void resetTrim() {
        trimDeg = 0;
    }

    /** For tests only: send this servo position (0 to 1) directly, ignoring the angle. Pass null to go back to angles. */
    public void setRawPositionForTest(Double position) {
        testPosition = position;
    }

    /** Target plus trim, clipped to MIN_LAUNCH_DEG..MAX_LAUNCH_DEG: the angle the servo is sent to. */
    public double commandedDeg() {
        // MAX first, then MIN: if Panels has MIN above MAX, MIN wins and nothing breaks.
        return Math.max(Math.min(targetDeg + trimDeg, MAX_LAUNCH_DEG), MIN_LAUNCH_DEG);
    }

    /** Where the hood probably is now. It moves toward commandedDeg() at SPEED_DEG_PER_SEC. */
    public double estimatedDeg() {
        return estimating ? estimatedDeg : commandedDeg();
    }

    public boolean atTarget() {
        if (!estimating || sinceEstimateStart.seconds() < fullRangeSeconds()) return false;
        return Math.abs(estimatedDeg - commandedDeg()) <= ON_ANGLE_DEG;
    }

    /** The position last sent to the servo, 0 to 1. NaN before the first update(). */
    public double servoPosition() {
        return position;
    }

    /**
     * True when the last update() wanted a servo position past 0 to 1. The servo stops at its
     * end, short of the angle, but atTarget() still turns true, so shots would go at the wrong
     * angle. The mapping numbers (ANCHOR_DEG, SERVO_AT_ANCHOR, GEAR_RATIO, REVERSED) are off.
     */
    public boolean servoAtEnd() {
        return atEnd;
    }

    /** Works out the servo position from the statics, writes it, and moves the estimate. Call it once per loop. */
    public void update() {
        double dt = Range.clip(loopTimer.seconds(), 0, MAX_LOOP_SECONDS);
        loopTimer.reset();

        if (testPosition != null) {
            estimating = false;
            atEnd = false;
            if (Double.isFinite(testPosition)) {
                position = Range.clip(testPosition, 0, 1);
                servo.setPosition(position);
            }
            return;
        }

        double commanded = commandedDeg();
        if (!estimating) {
            // At power-on (or after a test moved the servo directly) the hood could be anywhere,
            // so the estimate starts at the commanded angle and atTarget() stays false as long
            // as a move across the whole range would take.
            estimating = true;
            sinceEstimateStart.reset();
            estimatedDeg = commanded;
        } else {
            double step = speed() * dt;
            estimatedDeg += Range.clip(commanded - estimatedDeg, -step, step);
        }

        double wanted = rawPosition(commanded);
        atEnd = wanted < 0 || wanted > 1;
        // Never send NaN (a bad Panels value): the servo keeps its last position.
        if (Double.isFinite(wanted)) {
            position = Range.clip(wanted, 0, 1);
            servo.setPosition(position);
        }
    }

    /** The mapping from the top comment, before clipping to 0..1. NaN if SERVO_RANGE_DEG is unusable. */
    private static double rawPosition(double launchDeg) {
        if (SERVO_RANGE_DEG < MIN_SERVO_RANGE_DEG) return Double.NaN;
        double change = (launchDeg - ANCHOR_DEG) * GEAR_RATIO / SERVO_RANGE_DEG;
        return REVERSED ? SERVO_AT_ANCHOR - change : SERVO_AT_ANCHOR + change;
    }

    /**
     * The launch angle the mapping gives for a servo position: the top comment's mapping
     * turned around. Coax Hood Test uses it when leaving raw servo. NaN if GEAR_RATIO is 0
     * or SERVO_RANGE_DEG is unusable.
     */
    public static double launchDegAt(double servoPosition) {
        if (GEAR_RATIO == 0 || SERVO_RANGE_DEG < MIN_SERVO_RANGE_DEG) return Double.NaN;
        double change = REVERSED ? SERVO_AT_ANCHOR - servoPosition : servoPosition - SERVO_AT_ANCHOR;
        return ANCHOR_DEG + change * SERVO_RANGE_DEG / GEAR_RATIO;
    }

    private static double speed() {
        return Math.max(SPEED_DEG_PER_SEC, MIN_SPEED_DEG_PER_SEC);
    }

    private static double fullRangeSeconds() {
        return Math.abs(MAX_LAUNCH_DEG - MIN_LAUNCH_DEG) / speed();
    }

    public void addTelemetry(Telemetry telemetry) {
        if (testPosition != null) {
            telemetry.addData("Hood servo position", "%.3f, set directly by the test (not an angle)", position);
            return;
        }
        telemetry.addData("Hood angle", "%.1f -> %.1f deg, %s", estimatedDeg(), commandedDeg(), atTarget() ? "at target" : "moving");
        telemetry.addData("Hood target / trim", "%.1f / %+.1f deg", targetDeg, trimDeg);
        telemetry.addData("Hood servo position", "%.3f", position);
        double wanted = rawPosition(commandedDeg());
        if (!Double.isFinite(wanted)) {
            telemetry.addData("HOOD SERVO NOT SENT", "SERVO_RANGE_DEG must be at least %.0f and every hood value a number. The servo keeps its last position.", MIN_SERVO_RANGE_DEG);
        } else if (wanted < 0 || wanted > 1) {
            telemetry.addData("Hood servo at its end", "%.3f is past 0 to 1, so the hood can't reach this angle. Check ANCHOR_DEG, SERVO_AT_ANCHOR, GEAR_RATIO and REVERSED.", wanted);
        }
    }
}
