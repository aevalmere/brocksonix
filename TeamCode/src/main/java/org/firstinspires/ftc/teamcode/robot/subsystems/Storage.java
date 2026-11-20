package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.hardware.HardwareNames;
import org.firstinspires.ftc.teamcode.util.Debouncer;

/**
 * Sees POLLEN on the rail with three break beams, at slots 1, 3 and 4.
 *
 * Slot 1 is against the door, slot 2 behind it (no beam), slot 3 behind that, and the
 * "full" beam at slot 4 only stays blocked when all 4 are stored. A ball entering also
 * crosses the full beam, so full means blocked for a while (FULL_HOLD_MS), and only with
 * the slot 1 and slot 3 beams blocked too.
 *
 * Slot 2 has no beam, so a ball there is never seen. While collecting, the transfer pulls
 * balls up to the door and the intake pushes the rest behind them, so they stack from slot 1:
 *   0 balls: no beam.  1 or 2: slot 1 only (the two look the same).
 *   3: slots 1 and 3.  4: slots 1, 3 and full.
 * While the rail is stopped (a burst, the door moving) balls can be spread out, and a ball
 * resting at slot 2 with slot 1 clear is invisible. Each method below says only what the
 * beams really know.
 */
@Configurable
public class Storage {
    /** Checked on the robot in Breakbeam Test: the Adafruit receivers read low when the beam is blocked. */
    public static boolean BLOCKED_READS_LOW = true;
    // TODO(3): Raise (in ms) if the ball count flickers.
    public static double DEBOUNCE_MS = 30;
    // TODO(3): Feed balls one at a time. Full must appear only on the 4th.
    public static double FULL_HOLD_MS = 200;

    /** The three beam names, door end first. */
    private static final String[] BEAM_NAMES = {HardwareNames.BEAM_SLOT_1, HardwareNames.BEAM_SLOT_3, HardwareNames.BEAM_FULL};

    private final DigitalChannel slot1Beam, slot3Beam, fullBeam;
    private final Debouncer slot1 = new Debouncer(false);
    private final Debouncer slot3 = new Debouncer(false);
    private final Debouncer fullBlocked = new Debouncer(false);
    private final ElapsedTime fullBlockedFor = new ElapsedTime();
    private boolean full = false;

    /** Needs all three beams in the configuration: a missing one stops the OpMode at init. */
    public Storage(HardwareMap hardwareMap) {
        slot1Beam = input(hardwareMap, HardwareNames.BEAM_SLOT_1);
        slot3Beam = input(hardwareMap, HardwareNames.BEAM_SLOT_3);
        fullBeam = input(hardwareMap, HardwareNames.BEAM_FULL);
    }

    private static DigitalChannel input(HardwareMap hardwareMap, String name) {
        DigitalChannel channel = hardwareMap.get(DigitalChannel.class, name);
        channel.setMode(DigitalChannel.Mode.INPUT);
        return channel;
    }

    /**
     * For test OpModes that run without some beams: the names missing from the configuration,
     * comma separated, or "" when all three are there (then new Storage(hardwareMap) is safe).
     */
    public static String missingBeams(HardwareMap hardwareMap) {
        StringBuilder missing = new StringBuilder();
        for (String name : BEAM_NAMES) {
            if (hardwareMap.tryGet(DigitalChannel.class, name) != null) continue;
            if (missing.length() > 0) missing.append(", ");
            missing.append(name);
        }
        return missing.toString();
    }

    public void update() {
        slot1.update(isBlocked(slot1Beam), DEBOUNCE_MS);
        slot3.update(isBlocked(slot3Beam), DEBOUNCE_MS);

        boolean wasBlocked = fullBlocked.get();
        boolean blocked = fullBlocked.update(isBlocked(fullBeam), DEBOUNCE_MS);
        if (blocked && !wasBlocked) fullBlockedFor.reset();
        // Full also needs the two beams ahead. Otherwise a slow ball that stops in the full beam
        // with room in front of it would stop the intake for good: nothing would move it on.
        full = blocked && fullBlockedFor.milliseconds() >= FULL_HOLD_MS && slot1.get() && slot3.get();
    }

    /** Public so the test OpModes show a beam with the same rule (and flag) the ball count uses. */
    public static boolean isBlocked(DigitalChannel beam) {
        return beam.getState() != BLOCKED_READS_LOW;
    }

    /** A ball sits against the door, the next one to shoot. This is what a TeleOp burst needs to start. */
    public boolean slot1Occupied() {
        return slot1.get();
    }

    /** A ball at slot 3. With balls stacked from the door, that means at least 3. */
    public boolean slot3Occupied() {
        return slot3.get();
    }

    /** All 4 stored: the full beam has stayed blocked for FULL_HOLD_MS, with slots 1 and 3 blocked too. */
    public boolean isFull() {
        return full;
    }

    /**
     * No beam sees a ball. A ball resting at slot 2 with slot 1 clear is not seen, which can
     * only happen while the rail is stopped: as soon as it collects or feeds, that ball
     * reaches slot 1 (shootAll's LAST_BALL_MS covers it while feeding).
     */
    public boolean isEmpty() {
        return isEmpty(slot1.get(), slot3.get(), full);
    }

    static boolean isEmpty(boolean slot1, boolean slot3, boolean full) {
        return !slot1 && !slot3 && !full;
    }

    /** "0", "1-2", "2-3" or "4": the balls the beams see, and maybe one more at slot 2. */
    public String countLabel() {
        return countLabel(slot1.get(), slot3.get(), full);
    }

    /**
     * With balls stacked from the door, "1-2" is 1 or 2 balls and "2-3" is 3 (2 only while
     * they are spread out). Slot 3 alone, slot 1 clear, also shows "1-2" (slot 2 may hold one).
     */
    static String countLabel(boolean slot1, boolean slot3, boolean full) {
        if (full) return "4";
        int seen = (slot1 ? 1 : 0) + (slot3 ? 1 : 0);
        if (seen == 0) return "0";
        return seen + "-" + (seen + 1);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Balls", countLabel());
        telemetry.addData("Beams (slot1, slot3, full)", "%b, %b, %b", slot1.get(), slot3.get(), fullBlocked.get());
    }
}
