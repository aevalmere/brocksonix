package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.horizon36596.simloop.opmode.OpModePhase;
import org.horizon36596.simloop.opmode.OpModeRunResult;
import org.horizon36596.simloop.opmode.OpModeRunner;
import org.horizon36596.simloop.simclock.SimClockLoader;
import org.horizon36596.zenith.AutoFile;
import org.horizon36596.zenith.AutoSource;
import org.horizon36596.zenith.Json;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.Locale;

/**
 * Runs a Zenith auto file through our real code, headless: TracedZenithAuto (ZenithAuto
 * and TeamZenithRobot), Robot, RobotCommands, Ivy and Pedro's follower, on RobotSim.
 * It writes TeamCode/build/sim/{auto}.trace.json, which Zenith reads (zenith sim).
 *
 * simulatesTheNamedAuto is what `zenith sim <auto>` runs (zenith.json, "sim"):
 *   ./gradlew :TeamCode:testDebugUnitTest --tests "*ZenithAutoHeadlessTest*" -Dzenith.auto=template
 * Add -Dzenith.alliance=BLUE (or set ZENITH_ALLIANCE=BLUE) to run it for the other alliance.
 * It only fails when the run itself breaks (an exception, a thread), not when the auto
 * does badly: Zenith's report is where that shows.
 *
 * Without -Dzenith.auto, the normal test run checks the template auto instead: it
 * scores its 4 preloads for both alliances, and gives the same trace three times in a row
 * in one JVM, so no static (Scheduler, RobotState, Panels tunables, NamedCommands)
 * carries from one run to the next.
 *
 * Clocks: forTeamClass loads our classes, Pedro (Ivy included) and the Zenith runtime
 * again for each run and moves their clock reads (ElapsedTime, System.nanoTime,
 * currentTimeMillis, Thread.sleep) onto the sim clock, 20 ms a tick. So Ivy's waitMs,
 * our ElapsedTime timers, DriveRoute.NOW, Zenith's timeouts and Pedro's follower all run
 * on sim time, as fast as the computer goes. A thread the OpMode starts is not stepped,
 * so failOnOpModeThreads fails the run if one appears (AgateFlow's planner would).
 */
class ZenithAutoHeadlessTest {
    private static final double TICK = OpModeRunner.DEFAULT_TICK_SECONDS;
    /** 2 s of init: Pedro's PinpointLocalizer waits 0.5 s on start-up, then the alliance is picked. */
    private static final int INIT_LOOP_TICKS = 100;
    /** D-pad up is pressed on these init-loop ticks: once for RED, the second time for BLUE (AllianceSelector). */
    private static final int FIRST_PRESS = 40, SECOND_PRESS = 60, PRESS_TICKS = 3;
    /** The 30 s auto and 2 s to spare. The run ends sooner, when the routine finishes. */
    private static final int ACTIVE_TICKS = (int) Math.round(32 / TICK);

    @Test
    void simulatesTheNamedAuto() throws IOException {
        String auto = System.getProperty("zenith.auto");
        assumeTrue(auto != null, "no -Dzenith.auto: the template checks below run instead");
        Run run = simulate(auto, System.getProperty("zenith.alliance"), traceFile(auto));
        assertTrue(run.traceFile.isFile(), "no trace was written at " + run.traceFile);
    }

    @Test
    void templateScoresItsPreloadsForBothAlliances() throws IOException {
        assumeTrue(System.getProperty("zenith.auto") == null, "-Dzenith.auto names one auto to run instead");
        AutoFile template = AutoFile.load(AutoSource.directory(autosDir()), "template");

        Run red = simulate("template", "RED", traceFile("template"));
        checkScored(red, template.steps.size());
        assertNear(-34.75, -22.75, lastTruth(red), "red ends at templateShoot");

        Run blue = simulate("template", "BLUE", new File(simDir(), "template.blue.trace.json"));
        checkScored(blue, template.steps.size());
        // Blue really drove on its own side: RobotSim's truth is the field as it is...
        assertNear(34.75, 22.75, blue.sim.truth, "blue ends at templateShoot spun about the center");
        // ...and its trace is mirrored back into the file's frame, so it lies over red's.
        assertNear(-34.75, -22.75, lastTruth(blue), "blue's trace ends at templateShoot, mirrored back");
        assertTrue(blue.trace.contains("\"alliance\": \"BLUE\""), "blue trace names its alliance");
    }

    @Test
    void sameTraceThreeTimesInOneJvm() throws IOException {
        assumeTrue(System.getProperty("zenith.auto") == null, "-Dzenith.auto names one auto to run instead");
        String first = simulate("template", "RED", new File(simDir(), "template.repeat1.trace.json")).trace;
        for (int i = 2; i <= 3; i++) {
            String again = simulate("template", "RED", new File(simDir(), "template.repeat" + i + ".trace.json")).trace;
            assertEquals(first, again, "run " + i + " differs from run 1: something carried over between runs");
        }
    }

    /** Every step ran to its end, all 4 balls were launched and none spilled, and the routine finished early. */
    private static void checkScored(Run run, int steps) {
        assertTrue(run.result.endedBeforeStop(), "the routine never finished in 30 s");
        assertEquals(4, run.sim.launched(), "balls launched");
        assertEquals(0, run.sim.held(), "balls still held");
        assertEquals(0, run.sim.spilled(), "balls spilled out of the intake");
        Json trace = Json.parse(run.trace, run.traceFile.getName());
        assertEquals(steps, trace.get("steps").size(), "steps recorded");
        for (Json step : trace.get("steps").items()) {
            assertFalse(step.get("interrupted").bool(), step.str("id") + " was cut off (timeout)");
        }
        // What Zenith's report reads: launches added up over the ledger, held from its last row.
        int launches = 0;
        for (Json row : trace.get("ledger").items()) launches += (int) row.optNum("launches", 0);
        assertEquals(4, launches, "launches in the trace's ledger");
        Json ledger = trace.get("ledger");
        assertEquals(0, ledger.at(ledger.size() - 1).integer("held"), "held at the end of the trace");
    }

    /** Where the robot really was when the routine ended: the trace's last truth pose, in the file's frame. */
    private static double[] lastTruth(Run run) {
        Json truthPoses = Json.parse(run.trace, run.traceFile.getName()).get("truthPoses");
        Json last = truthPoses.at(truthPoses.size() - 1);
        return new double[] {last.at(1).num(), last.at(2).num()};
    }

    private static void assertNear(double x, double y, double[] actual, String what) {
        double off = Math.hypot(actual[0] - x, actual[1] - y);
        assertTrue(off < 2, what + ": " + String.format(Locale.ROOT, "(%.2f, %.2f), %.2f in off", actual[0], actual[1], off));
    }

    /** One sim run and what it left. */
    static final class Run {
        final RobotSim sim;
        final OpModeRunResult result;
        final File traceFile;
        final String trace;

        Run(RobotSim sim, OpModeRunResult result, File traceFile) throws IOException {
            this.sim = sim;
            this.result = result;
            this.traceFile = traceFile;
            this.trace = new String(Files.readAllBytes(traceFile.toPath()), StandardCharsets.UTF_8);
        }
    }

    /**
     * Runs one auto file from autos/ (the files Zenith edits, not the deployed copies) and
     * writes its trace. alliance is "RED", "BLUE" or null for the file's own.
     */
    static Run simulate(String auto, String alliance, File traceFile) throws IOException {
        File autosDir = autosDir();
        AutoFile spec = AutoFile.load(AutoSource.directory(autosDir), auto);
        String running = alliance == null ? spec.alliance : alliance.trim().toUpperCase(Locale.ROOT);
        if (!running.equals("RED") && !running.equals("BLUE")) {
            throw new IllegalArgumentException("zenith.alliance must be RED or BLUE, not " + alliance);
        }

        RobotSim sim = new RobotSim();
        // The robot really starts where the file says, on the running alliance's side.
        Pose start = spec.startPose;
        if (spec.mirrorsInto(running)) sim.place(-start.x(), -start.y(), start.heading() + Math.PI);
        else sim.place(start.x(), start.y(), start.heading());
        sim.preload(preload(autosDir, auto));

        if (traceFile.exists() && !traceFile.delete()) throw new IOException("can't replace " + traceFile);
        OpMode.blackboard.put(TracedZenithAuto.AUTO, auto);
        OpMode.blackboard.put(TracedZenithAuto.AUTOS_DIR, autosDir.getAbsolutePath());
        OpMode.blackboard.put(TracedZenithAuto.TRACE, traceFile.getAbsolutePath());
        OpMode.blackboard.put(TracedZenithAuto.TICK, TICK);
        OpMode.blackboard.put(TracedZenithAuto.TRUTH, sim.truth);
        OpMode.blackboard.put(TracedZenithAuto.ENDED, false);

        final int presses = running.equals("RED") ? 1 : 2;
        OpModeRunResult result = OpModeRunner.forTeamClass(TracedZenithAuto.class, sim.hardwareMap)
                // Pedro (with Ivy) and Zenith read the clock too, so they go on the sim clock with our code.
                .teamPackages("org.firstinspires.ftc.teamcode.", SimClockLoader.PEDRO_PATHING, "org.horizon36596.zenith.")
                .keepBlackboard(true)
                .tickSeconds(TICK)
                .initLoopTicks(INIT_LOOP_TICKS)
                .activeTicks(ACTIVE_TICKS)
                .gamepads((phase, tick, gamepad1, gamepad2) -> {
                    if (phase != OpModePhase.INIT_LOOP) return;
                    boolean first = tick >= FIRST_PRESS && tick < FIRST_PRESS + PRESS_TICKS;
                    boolean second = tick >= SECOND_PRESS && tick < SECOND_PRESS + PRESS_TICKS;
                    gamepad1.dpad_up = first || (presses == 2 && second);
                })
                // On the robot the SDK stops every motor when the OpMode ends. SimLoop keeps ticking to
                // the last tick with the motors as they were, so the world stops with the OpMode here.
                .eachTick(dt -> {
                    if (!Boolean.TRUE.equals(OpMode.blackboard.get(TracedZenithAuto.ENDED))) sim.step(dt);
                })
                .failOnOpModeThreads(true)
                .run();

        Run run = new Run(sim, result, traceFile);
        System.out.printf(Locale.ROOT, "%s %s: %s at %.2f s, %d launched, %d held, %d spilled. Trace: %s%n",
                auto, running, result.endedBeforeStop() ? "routine finished" : "30 s ran out",
                Json.parse(run.trace, traceFile.getName()).num("simTimeS"),
                sim.launched(), sim.held(), sim.spilled(), traceFile);
        for (String warning : result.warnings()) System.out.println("SimLoop: " + warning);
        return run;
    }

    /** How many POLLEN the file says the robot starts with (start.holds.pollen). */
    private static int preload(File autosDir, String auto) throws IOException {
        String name = auto + ".auto.json";
        try (InputStream in = new FileInputStream(new File(autosDir, name))) {
            Json holds = Json.parse(in, name).get("start").opt("holds");
            return holds == null ? 0 : (int) holds.optNum("pollen", 0);
        }
    }

    static File traceFile(String auto) {
        return new File(simDir(), auto + ".trace.json");
    }

    /** TeamCode/build/sim, where zenith.json's "sim.trace" looks. */
    static File simDir() {
        return new File(repoRoot(), "TeamCode/build/sim");
    }

    static File autosDir() {
        return new File(repoRoot(), "autos");
    }

    /** The folder with zenith.json in it. Gradle runs the tests in TeamCode/. */
    static File repoRoot() {
        File dir = new File(System.getProperty("user.dir")).getAbsoluteFile();
        while (dir != null && !new File(dir, "zenith.json").isFile()) dir = dir.getParentFile();
        if (dir == null) throw new IllegalStateException("no zenith.json above " + System.getProperty("user.dir"));
        return dir;
    }
}
