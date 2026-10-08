package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.hardware.HardwareNames;
import org.firstinspires.ftc.teamcode.util.Debouncer;

/**
 * Counts POLLEN on the rail with three break beams.
 *
 * Slot 1 is against the door, slot 2 behind it, slot 3 has no beam, and the
 * "full" beam at slot 4 only stays blocked when all 4 are stored. A ball
 * entering also crosses the full beam, so full means blocked for a while.
 */
@Configurable
public class Storage {
    // TODO(3): Storage Test: each beam must show true when blocked by hand. Flip if backwards.
    /** Adafruit receivers usually read low when the beam is blocked. */
    public static boolean BLOCKED_READS_LOW = true;
    // TODO(3): Raise (in ms) if the ball count flickers.
    public static double DEBOUNCE_MS = 30;
    // TODO(3): Feed balls one at a time. Full must appear only on the 4th.
    public static double FULL_HOLD_MS = 150;

    private final DigitalChannel slot1Beam, slot2Beam, fullBeam;
    private final Debouncer slot1 = new Debouncer(false);
    private final Debouncer slot2 = new Debouncer(false);
    private final Debouncer fullBlocked = new Debouncer(false);
    private final ElapsedTime fullBlockedFor = new ElapsedTime();
    private boolean full = false;

    public Storage(HardwareMap hardwareMap) {
        slot1Beam = input(hardwareMap, HardwareNames.BEAM_SLOT_1);
        slot2Beam = input(hardwareMap, HardwareNames.BEAM_SLOT_2);
        fullBeam = input(hardwareMap, HardwareNames.BEAM_FULL);
    }

    private static DigitalChannel input(HardwareMap hardwareMap, String name) {
        DigitalChannel channel = hardwareMap.get(DigitalChannel.class, name);
        channel.setMode(DigitalChannel.Mode.INPUT);
        return channel;
    }

    public void update() {
        slot1.update(isBlocked(slot1Beam), DEBOUNCE_MS);
        slot2.update(isBlocked(slot2Beam), DEBOUNCE_MS);

        boolean wasBlocked = fullBlocked.get();
        boolean blocked = fullBlocked.update(isBlocked(fullBeam), DEBOUNCE_MS);
        if (blocked && !wasBlocked) fullBlockedFor.reset();
        full = blocked && fullBlockedFor.milliseconds() >= FULL_HOLD_MS;
    }

    private static boolean isBlocked(DigitalChannel beam) {
        return beam.getState() != BLOCKED_READS_LOW;
    }

    public boolean slot1Occupied() {
        return slot1.get();
    }

    public boolean slot2Occupied() {
        return slot2.get();
    }

    public boolean isFull() {
        return full;
    }

    public boolean isEmpty() {
        return !slot1.get() && !slot2.get() && !full;
    }

    /** A shot or pass can only start with at least 2 balls. */
    public boolean hasAtLeastTwo() {
        return full || (slot1.get() && slot2.get());
    }

    /** "0", "1", "2-3" or "4". Slot 3 has no beam, so 2 and 3 look the same. */
    public String countLabel() {
        if (full) return "4";
        if (slot1.get() && slot2.get()) return "2-3";
        if (slot1.get() || slot2.get()) return "1";
        return "0";
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Balls", countLabel());
        telemetry.addData("Beams (slot1, slot2, full)", "%b, %b, %b", slot1.get(), slot2.get(), fullBlocked.get());
    }
}
