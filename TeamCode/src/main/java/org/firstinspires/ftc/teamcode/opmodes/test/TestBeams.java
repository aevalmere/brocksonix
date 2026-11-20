package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.hardware.HardwareNames;
import org.firstinspires.ftc.teamcode.robot.subsystems.Storage;

/**
 * The three break beams for the test OpModes (Continuous Test, Breakbeam Test), in the order a
 * ball passes them: slot 4 (intake end), slot 3, slot 1 (door). There is no slot 2 beam.
 * Each beam is optional here, unlike in Storage.
 */
final class TestBeams {
    /** Configuration names, in ball-travel order. */
    static final String[] NAMES = {HardwareNames.BEAM_FULL, HardwareNames.BEAM_SLOT_3, HardwareNames.BEAM_SLOT_1};
    /** The slot each beam sits at, same order as NAMES. */
    static final int[] SLOTS = {4, 3, 1};

    private TestBeams() {
    }

    /** "Beam slot 4 (beamFull)" for beam i of NAMES. */
    static String label(int i) {
        return "Beam slot " + SLOTS[i] + " (" + NAMES[i] + ")";
    }

    /** All three beams in NAMES order; a beam missing from the configuration is null. */
    static DigitalChannel[] find(HardwareMap hardwareMap) {
        DigitalChannel[] beams = new DigitalChannel[NAMES.length];
        for (int i = 0; i < NAMES.length; i++) beams[i] = optional(hardwareMap, NAMES[i]);
        return beams;
    }

    /** The beam as a digital input, or null when it isn't in the configuration (tryGet doesn't throw). */
    static DigitalChannel optional(HardwareMap hardwareMap, String name) {
        DigitalChannel beam = hardwareMap.tryGet(DigitalChannel.class, name);
        if (beam != null) beam.setMode(DigitalChannel.Mode.INPUT);
        return beam;
    }

    /** Live and not debounced, so every pass shows. The raw level is there to check BLOCKED_READS_LOW. */
    static String text(DigitalChannel beam) {
        if (beam == null) return "not configured";
        return (Storage.isBlocked(beam) ? "BLOCKED" : "clear") + (beam.getState() ? " (reads high)" : " (reads low)");
    }
}
