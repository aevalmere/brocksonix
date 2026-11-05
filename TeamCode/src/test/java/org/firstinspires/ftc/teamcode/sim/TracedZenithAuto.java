package org.firstinspires.ftc.teamcode.sim;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Angles;
import org.firstinspires.ftc.teamcode.zenith.TeamZenithRobot;
import org.firstinspires.ftc.teamcode.zenith.ZenithAuto;
import org.firstinspires.ftc.teamcode.zenith.ZenithFrame;
import org.horizon36596.zenith.AutoSource;
import org.horizon36596.zenith.AutoTrace;
import org.horizon36596.zenith.ivy.AutoFromFile;
import org.horizon36596.zenith.ivy.AutoTraceWriter;
import org.horizon36596.zenith.ivy.ZenithRobot;

import java.io.File;
import java.io.IOException;
import java.io.UncheckedIOException;

/**
 * A Zenith auto OpMode as zenith deploy writes one (a ZenithAuto naming a file and
 * TeamZenithRobot), with three changes for the sim:
 * - Robot gets SimFollower's follower (buildRobot), since Constants.create() has none yet.
 * - Every step is wrapped to record when it starts and ends. It wraps
 *   TeamZenithRobot.step, so paths keep their default timeout.
 * - Each loop records where the follower thinks the robot is, where it really is
 *   (RobotSim.truth), the balls held and launched, and each shot Robot counts. Once
 *   the routine has finished it stops itself, and the trace is written.
 *
 * SimLoop builds this class itself, in a class loader of its own (OpModeRunner.forTeamClass),
 * so ZenithAutoHeadlessTest hands it everything through OpMode.blackboard, as SDK and
 * JDK types only. The alliance is picked in init with the D-pad, as on the robot.
 *
 * Trace poses are in Zenith's frame (ZenithFrame), and in the file's alliance: a run for
 * the other alliance is mirrored back, so Zenith lays it over the auto as drawn.
 */
public class TracedZenithAuto extends ZenithAuto {
    // OpMode.blackboard keys.
    static final String AUTO = "sim.auto";
    static final String AUTOS_DIR = "sim.autosDir";
    static final String TRACE = "sim.trace";
    static final String TICK = "sim.tickSeconds";
    static final String TRUTH = "sim.truth";
    /** Set true here once runOpMode() returns, so the test stops the world with the OpMode. */
    static final String ENDED = "sim.ended";

    private AutoTraceWriter trace;
    private double[] truth;
    private Robot simRobot;
    private boolean mirrorBack;
    private long startNanos;
    private int lastHeld = -1, lastLaunched = 0, lastShotsCounted = 0;

    @Override
    protected String autoName() {
        return (String) blackboard.get(AUTO);
    }

    @Override
    protected ZenithRobot createRobot() {
        return new TracedRobot();
    }

    @Override
    protected Robot buildRobot() {
        simRobot = new Robot(hardwareMap, SimFollower.create(hardwareMap));
        return simRobot;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        trace = new AutoTraceWriter(autoName(), (Double) blackboard.get(TICK));
        trace.capability(AutoTrace.LEDGER);
        truth = (double[]) blackboard.get(TRUTH);
        setAutoSource(AutoSource.directory(new File((String) blackboard.get(AUTOS_DIR))));
        try {
            super.runOpMode();
        } finally {
            blackboard.put(ENDED, true);
            try {
                trace.write(new File((String) blackboard.get(TRACE)));
            } catch (IOException e) {
                throw new UncheckedIOException(e);
            }
        }
    }

    /** Seconds since start, on the sim clock (SimLoop moves System.nanoTime onto it). */
    private double seconds() {
        return (System.nanoTime() - startNanos) / 1e9;
    }

    private void record() {
        Pose pose = simRobot.drive.pose();
        double[] seen = fileFrame(pose.x() - ZenithFrame.CENTER, pose.y() - ZenithFrame.CENTER, pose.heading());
        trace.pose(seen[0], seen[1], seen[2]);
        double[] real = fileFrame(truth[0], truth[1], truth[2]);
        trace.truthPose(real[0], real[1], real[2]);

        // A ledger row holds the balls held now and the launches since the row before (Zenith adds those up).
        int held = (int) truth[3], launched = (int) truth[4];
        if (held != lastHeld || launched != lastLaunched) {
            trace.ledger().held(held).launches(launched - lastLaunched);
            lastHeld = held;
            lastLaunched = launched;
        }
        // Robot's own shot counter (FireControl's RPM dips), to set beside the balls really launched.
        for (; lastShotsCounted < simRobot.shotsFired(); lastShotsCounted++) trace.event("shotCounted");
    }

    /** A pose in Zenith's frame, mirrored back into the file's alliance when the run is for the other one. */
    private double[] fileFrame(double x, double y, double heading) {
        if (mirrorBack) return new double[] {-x, -y, Angles.wrapRadians(heading + Math.PI)};
        return new double[] {x, y, Angles.wrapRadians(heading)};
    }

    private final class TracedRobot extends TeamZenithRobot {
        @Override
        public Command step(String id, Command body) {
            return trace.traced(id, super.step(id, body));
        }

        @Override
        public void onStart(AutoFromFile opMode) {
            super.onStart(opMode);
            startNanos = System.nanoTime();
            mirrorBack = spec.mirrorsInto(runningAlliance());
            trace.tick(0);
            trace.event("alliance", runningAlliance());
            record();
        }

        /** Before Scheduler.execute(), so a step that starts or ends this loop gets this loop's time. */
        @Override
        public void periodic(AutoFromFile opMode) {
            trace.tick(seconds());
            super.periodic(opMode);
        }

        @Override
        public void afterLoop(AutoFromFile opMode) {
            super.afterLoop(opMode);
            record();
            if (!Scheduler.isRunning(routine())) requestOpModeStop();
        }
    }
}
