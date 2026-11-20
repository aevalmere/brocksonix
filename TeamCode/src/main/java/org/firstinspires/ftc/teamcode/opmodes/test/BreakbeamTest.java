package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.hardware.BulkReads;
import org.firstinspires.ftc.teamcode.robot.subsystems.Storage;

/**
 * Shows only the break beams, for feeding balls through by hand and checking wiring and
 * polarity. Moves nothing and needs only the beams in the configuration (no Pinpoint, no
 * motors); each beam is optional and shows "not configured" when missing.
 *
 * Per beam, in the order a ball passes them (slot 4, 3, 1): BLOCKED or clear with the raw
 * level, how long it has been that way, how many times it was blocked since Start, and how
 * long the last and the shortest block lasted, so a quick pass the eye misses still shows.
 * A block shorter than Storage.DEBOUNCE_MS is one the ball count never sees.
 * Blocked uses Storage.isBlocked, the same rule and flag (BLOCKED_READS_LOW) the count uses.
 *
 * Cross: zero the counts.
 */
@TeleOp(name = "Breakbeam Test", group = "Test")
public class BreakbeamTest extends LinearOpMode {

    /** One beam's live state and its pass history. */
    private static class BeamWatch {
        final DigitalChannel beam;
        boolean blocked;
        double changedAtMs;
        int blocks;
        double lastBlockMs = -1;     // -1: no finished block yet
        double shortestBlockMs = -1;

        BeamWatch(DigitalChannel beam) {
            this.beam = beam;
        }

        /** Takes the beam as it is now, without counting it as a pass. */
        void reset(double nowMs) {
            if (beam != null) blocked = Storage.isBlocked(beam);
            changedAtMs = nowMs;
            blocks = 0;
            lastBlockMs = -1;
            shortestBlockMs = -1;
        }

        void update(double nowMs) {
            if (beam == null) return;
            boolean now = Storage.isBlocked(beam);
            if (now == blocked) return;
            if (now) {
                blocks++; // rising edge: a ball just came in
            } else {
                lastBlockMs = nowMs - changedAtMs;
                if (shortestBlockMs < 0 || lastBlockMs < shortestBlockMs) shortestBlockMs = lastBlockMs;
            }
            blocked = now;
            changedAtMs = nowMs;
        }
    }

    @Override
    public void runOpMode() {
        Telemetry screen = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        // MANUAL bulk caching: the digital inputs only refresh when clear() is called.
        BulkReads bulkReads = new BulkReads(hardwareMap);
        DigitalChannel[] beams = TestBeams.find(hardwareMap);
        BeamWatch[] watches = new BeamWatch[beams.length];
        for (int i = 0; i < beams.length; i++) watches[i] = new BeamWatch(beams[i]);

        // Init only reads, so the live state is safe to show before Start.
        while (opModeInInit()) {
            bulkReads.clear();
            for (int i = 0; i < beams.length; i++) {
                screen.addData(TestBeams.label(i), beams[i] == null ? "not configured"
                        : "found, " + TestBeams.text(beams[i]));
            }
            addPolarityHint(screen);
            screen.addLine("Moves nothing. Press Start, then feed balls by hand.");
            screen.update();
        }

        waitForStart();
        // "WasPressed" remembers presses made during init, which would fire on the first loop.
        gamepad1.resetEdgeDetection();
        ElapsedTime clock = new ElapsedTime();
        ElapsedTime loopTimer = new ElapsedTime();
        bulkReads.clear();
        for (BeamWatch watch : watches) watch.reset(clock.milliseconds());

        while (opModeIsActive()) {
            double loopMs = loopTimer.milliseconds();
            loopTimer.reset();
            bulkReads.clear();
            double nowMs = clock.milliseconds();
            if (gamepad1.crossWasPressed()) {
                for (BeamWatch watch : watches) watch.reset(nowMs);
            }
            for (BeamWatch watch : watches) watch.update(nowMs);

            for (int i = 0; i < watches.length; i++) {
                BeamWatch watch = watches[i];
                if (watch.beam == null) {
                    screen.addData(TestBeams.label(i), "not configured");
                    continue;
                }
                screen.addData(TestBeams.label(i), "%s, %.1f s",
                        TestBeams.text(watch.beam), (nowMs - watch.changedAtMs) / 1000.0);
                screen.addData("  slot " + TestBeams.SLOTS[i] + " blocks", "%d, last %s, shortest %s",
                        watch.blocks, msText(watch.lastBlockMs), msText(watch.shortestBlockMs));
            }
            addPolarityHint(screen);
            screen.addData("Loop", "%.0f ms (a block shorter than this can be missed)", loopMs);
            screen.addLine("Cross: zero the counts.");
            screen.update();
        }
    }

    private static void addPolarityHint(Telemetry screen) {
        screen.addData("Storage.BLOCKED_READS_LOW", "%b (Panels page Storage)", Storage.BLOCKED_READS_LOW);
        screen.addLine("BLOCKED with nothing in the beam: that flag is wrong, flip it.");
    }

    private static String msText(double ms) {
        return ms < 0 ? "-" : String.format("%.0f ms", ms);
    }
}
