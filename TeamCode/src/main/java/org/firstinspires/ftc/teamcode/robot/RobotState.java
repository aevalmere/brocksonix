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
    // nanoTime only counts up. currentTimeMillis can jump when the Control Hub syncs its clock.
    private static long savedAtNanos = 0;

    /** Switching alliance drops the saved pose, since it is in the old alliance's frame. Picking the same one keeps it. */
    public static void setAlliance(Alliance picked) {
        if (picked != alliance) savedPose = null;
        alliance = picked;
    }

    public static void savePose(Pose pose) {
        savedPose = pose;
        savedAtNanos = System.nanoTime();
    }

    /** The saved pose if it's recent, otherwise null. */
    public static Pose recentPose() {
        boolean recent = savedPose != null && (System.nanoTime() - savedAtNanos) / 1e9 <= MAX_SAVED_AGE_SEC;
        return recent ? savedPose : null;
    }

    /** Clears the alliance from an old match, so it has to be picked again. */
    public static void forgetIfStale() {
        if (recentPose() == null) alliance = null;
    }
}
