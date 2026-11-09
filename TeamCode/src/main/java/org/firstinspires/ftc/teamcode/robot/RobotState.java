package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;

/**
 * Things that carry from one OpMode to the next (auto into teleop, a teleop
 * restart, or one test OpMode to the next). Static fields live until the robot
 * app restarts, which can be several matches later.
 *
 * There is one saved pose, with a flag for whether it is known to be right.
 * Match OpModes read it with recentPose(), so they only get a trusted pose saved
 * recently (auto just ran, or a quick restart), and the alliance is kept only
 * along with such a pose. Test OpModes read it with lastPose(), at any age and
 * trusted or not, so the pose follows the robot from test to test (TestPose).
 */
public final class RobotState {
    private RobotState() {}

    /** Long enough to cover the auto-to-teleop gap or a quick restart, too short to reach the next match. */
    public static final double MAX_SAVED_AGE_SEC = 60;

    /** Null until picked in init. Carries from auto into teleop. */
    public static Alliance alliance = null;
    public static Cell targetCell = Cell.FAR;

    private static Pose savedPose = null;
    private static boolean savedTrusted = false;
    // nanoTime only counts up. currentTimeMillis can jump when the Control Hub syncs its clock.
    private static long savedAtNanos = 0;

    /** Switching alliance drops the saved pose, since it is in the old alliance's frame. Picking the same one keeps it. */
    public static void setAlliance(Alliance picked) {
        if (picked != alliance) savedPose = null;
        alliance = picked;
    }

    /** Saves a pose that is known to be right. */
    public static void savePose(Pose pose) {
        savePose(pose, true);
    }

    /** @param trusted true if this pose is known to be right (fresh from auto, or just relocalized) */
    public static void savePose(Pose pose, boolean trusted) {
        savedPose = pose;
        savedTrusted = trusted;
        savedAtNanos = System.nanoTime();
    }

    /** The saved pose if it's trusted and recent, otherwise null. */
    public static Pose recentPose() {
        boolean recent = savedPose != null && savedTrusted
                && (System.nanoTime() - savedAtNanos) / 1e9 <= MAX_SAVED_AGE_SEC;
        return recent ? savedPose : null;
    }

    /** The saved pose at any age, trusted or not. Null if there is none. For the test OpModes. */
    public static Pose lastPose() {
        return savedPose;
    }

    /** True if lastPose() is known to be right. */
    public static boolean lastPoseTrusted() {
        return savedPose != null && savedTrusted;
    }

    /** Clears the alliance from an old match, so it has to be picked again. */
    public static void forgetIfStale() {
        if (recentPose() == null) alliance = null;
    }
}
