package org.firstinspires.ftc.teamcode.field;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.agateflow.AgateFlow;
import org.firstinspires.ftc.teamcode.util.SeparatingAxis;

/**
 * Zone fire's shooting zone (Robot.zoneFire): while any part of the robot is in
 * it, the robot fires as if RB were held, once the shot is ready (see Robot).
 *
 * Why a trapezoid: the up cell's opening is the cell's outer end, facing out
 * along y and tilted 30° (TELEOP_PLAN.md, "Field coordinates"). The places that
 * can see into it fan out from the opening and get wider with distance, like a
 * cone with its tip cut off. So the zone has its short side near the cell (meant
 * to be about the shot table's shortest distance away) and its long side toward
 * the wall.
 *
 * The corners are written for red with the FAR cell up. For the AUDIENCE cell
 * they are mirrored across the center line (y = SIZE / 2), and for blue across
 * x = SIZE / 2, since the blue HIVE is at SIZE - RED_HIVE_X. That is not
 * Alliance.fromRed's 180° spin: the cells keep their y for both alliances (see
 * Field.cellTarget).
 *
 * The robot is in the zone when any part of its outline overlaps it, touching
 * included: the rectangle AgateFlow.ROBOT_FRONT, ROBOT_BACK and ROBOT_HALF_WIDTH
 * around the turret center (flower intake up), the one PlanDrawing draws. The
 * overlap test is the separating axis theorem (util/SeparatingAxis).
 */
@Configurable
public final class ShootingZone {
    // TODO(12): Placeholder. Main TeleOp with zone fire on (driver 1 Triangle): shoot from all along the zone's edge and pull in any corner where shots miss. Any part of the robot in the zone fires, so keep the whole trapezoid inside where shots score. Keep it convex.
    /**
     * The 4 corners, {x, y} in field inches, for red with the FAR cell up, in
     * order around the shape (either way round). Placeholder values. The first two
     * are the short side, near the cell's opening at (58, 87.25): 16.75 in. from it
     * straight out (y 104) and 19.5 in. at the corners. That is closer than
     * ShotTables.SHOT_RPM's first row (20 in.), so near the short side the shot is
     * out of range and zone fire doesn't fire. The last two are the long side toward
     * the far wall. That is roughly ±35° either side of straight out from the cell.
     */
    public static double[][] RED_FAR_CORNERS = {
            {48, 104},
            {68, 104},
            {90, 135},
            {26, 135},
    };

    private ShootingZone() {}

    /** The zone for this alliance with this cell up, in field inches: x of each corner in [0], y in [1]. */
    public static double[][] corners(Alliance alliance, Cell cell) {
        int n = RED_FAR_CORNERS.length;
        double[] xs = new double[n];
        double[] ys = new double[n];
        for (int i = 0; i < n; i++) {
            double x = RED_FAR_CORNERS[i][0];
            double y = RED_FAR_CORNERS[i][1];
            xs[i] = alliance == Alliance.RED ? x : Field.SIZE - x;
            ys[i] = cell == Cell.FAR ? y : Field.SIZE - y;
        }
        return new double[][] {xs, ys};
    }

    /** The robot's outline at this pose, flower intake up, in field inches: x of each corner in [0], y in [1]. */
    public static double[][] robotOutline(Pose robot) {
        double front = AgateFlow.ROBOT_FRONT;
        double back = AgateFlow.ROBOT_BACK;
        double half = AgateFlow.ROBOT_HALF_WIDTH;
        // The corners in the robot's own frame: forward is toward the intake, left is +.
        double[] forward = {front, -back, -back, front};
        double[] left = {half, half, -half, -half};
        double cos = Math.cos(robot.heading());
        double sin = Math.sin(robot.heading());
        double[] xs = new double[4];
        double[] ys = new double[4];
        for (int i = 0; i < 4; i++) {
            xs[i] = robot.x() + forward[i] * cos - left[i] * sin;
            ys[i] = robot.y() + forward[i] * sin + left[i] * cos;
        }
        return new double[][] {xs, ys};
    }

    /** True if any part of the robot's outline at this pose is in the zone (touching counts). */
    public static boolean overlaps(Alliance alliance, Cell cell, Pose robot) {
        double[][] zone = corners(alliance, cell);
        double[][] outline = robotOutline(robot);
        return SeparatingAxis.overlaps(zone[0], zone[1], outline[0], outline[1]);
    }
}
