package org.firstinspires.ftc.teamcode.agateflow;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.util.PIDF;

/**
 * Learns how far off the time estimates are on the real robot, and corrects
 * them. After each leg that ran start to finish on its own plan and got to its
 * stop, DriveRoute reports the predicted and the real time; TIME_SCALE moves a
 * little toward real/predicted.
 *
 * TIME_SCALE is in Panels like every tunable, so you can watch it settle. It
 * lives until the robot app restarts, so copy it into the code when it has
 * settled over a few autos.
 */
@Configurable
public final class EtaCalibration {
    // TODO(15): After a few autos with AgateFlow, copy the learned TIME_SCALE from Panels into the code here.
    /** Real time / predicted time. Estimates are multiplied by this. */
    public static double TIME_SCALE = 1.0;
    /** Learn from each finished leg. Turn off to keep TIME_SCALE fixed. */
    public static boolean LEARN = true;
    /**
     * Smoothing for TIME_SCALE, like every smoothing knob here: 0 = none (it jumps to
     * each leg's real/predicted), closer to 1 = smoother but slower to learn. At 0.8
     * each leg moves it a fifth of the way.
     */
    public static double LEARN_FILTER = 0.8;
    /** Legs shorter than this (seconds predicted) are too noisy to learn from. */
    public static double MIN_LEARN_SECONDS = 0.5;

    private EtaCalibration() {}

    public static double correct(double predictedSeconds) {
        return predictedSeconds * TIME_SCALE;
    }

    /** A leg ran start to finish on its own plan and got to its stop. */
    public static void record(double predictedSeconds, double actualSeconds) {
        if (!LEARN || predictedSeconds < MIN_LEARN_SECONDS || actualSeconds <= 0) return;
        double ratio = actualSeconds / predictedSeconds;
        // A ratio this far off means something went wrong (bumped, stuck), not a model error.
        if (ratio < 0.5 || ratio > 2) return;
        TIME_SCALE = PIDF.smooth(TIME_SCALE, ratio, LEARN_FILTER);
    }
}
