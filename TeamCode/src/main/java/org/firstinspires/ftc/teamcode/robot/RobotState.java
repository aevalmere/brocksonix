package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;

/**
 * Things that carry from one OpMode to the next (auto into teleop, or a
 * teleop restart). Static fields live until the robot app restarts, which
 * can be several matches later, so the saved pose and alliance are only
 * kept if the pose was saved recently (auto just ran, or a quick restart).
 */
public final class RobotState {
    private RobotState() {}

    /** Long enough to cover the auto-to-teleop gap or a quick restart, too short to reach the next match. */
    public static final double MAX_SAVED_AGE_SEC = 60;

    /** Null until picked in init. Carries from auto into teleop. */
    public static Alliance alliance = null;
    public static Cell targetCell = Cell.FAR;

    private static Pose savedPose = null;
    private static long savedAtMs = 0;

    public static void savePose(Pose pose) {
        savedPose = pose;
        savedAtMs = System.currentTimeMillis();
    }

    /** The saved pose if it's recent, otherwise null. */
    public static Pose recentPose() {
        boolean recent = savedPose != null && (System.currentTimeMillis() - savedAtMs) / 1000.0 <= MAX_SAVED_AGE_SEC;
        return recent ? savedPose : null;
    }

    /** Clears the alliance from an old match, so it has to be picked again. */
    public static void forgetIfStale() {
        if (recentPose() == null) alliance = null;
    }
}
