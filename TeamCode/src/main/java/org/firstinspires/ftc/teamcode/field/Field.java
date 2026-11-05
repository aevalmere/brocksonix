package org.firstinspires.ftc.teamcode.field;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.util.Angles;

/**
 * Where things are on the BIOBUZZ field, in inches.
 *
 * Standing where the audience stands: (0, 0) is the near-left corner (red
 * alliance wall and audience wall), +x points toward the blue wall, +y away
 * from the audience. Heading 0 faces +x and counter-clockwise is positive.
 * The field is 141.5 x 141.5, so its center is (70.75, 70.75).
 *
 * This is exactly Pedro Visualizer's frame (visualizer.pedropathing.com, BIOBUZZ
 * field): same corner, same directions, same headings, same 141.5 size, and its
 * picture has red on the left and the audience at the bottom like ours. So a
 * point copies over 1:1, no scaling. Example: red's FLOWER on the far wall is at
 * (47.3, 141.5) in the visualizer and here.
 *
 * For blue, write the red pose and let Alliance.fromRed spin it. Don't use the
 * visualizer's "mirror" export: it flips left to right, but the BIOBUZZ field
 * is a 180° spin.
 *
 * Which number wins: the real field. The Field Acceptance Checklist where it
 * controls a dimension (events measure those), then the official field CAD (STEP
 * v26-27.2, which manual 9.1 calls the official field), then the Competition
 * Manual's figures (TU07, Section 9). A distance off a wall is measured from the
 * wall. Anything in the middle, or placed by tile seams or wall panel joints, is
 * at the CAD's distance from the center. A number already within 0.1 in. of that
 * stays. Zenith's field file (autos/field/biobuzz.field.json) uses the same numbers.
 *
 * The CAD field is 141.17 between the wall rails (the tiles fill it) and 141.35
 * between the wall panels; the manual says about 144. SIZE stays at Pedro
 * Visualizer's 141.5. See TELEOP_PLAN.md, "Field coordinates", for each number.
 */
@Configurable
public final class Field {
    public static final double SIZE = 141.5;

    /** Red HIVE center line: the field center minus 12.75 (Figure 9-10: the HIVES are 25.5 apart). Blue's is mirrored across the center. */
    public static double RED_HIVE_X = 58.0;
    /**
     * Center of the upward cell's opening when the far cell is up: 70.75 + 16.5.
     * The opening is the cell's outer end, tilted 30°, from 53.5 to 65.6 in. up
     * (Figures 9-9 to 9-11), and its center is 16.5 in. from the HIVE center along
     * y (official field CAD; Pedro Visualizer agrees).
     */
    public static double FAR_CELL_Y = 87.25;
    /** Center of the upward cell's opening when the audience cell is up: 70.75 - 16.5. */
    public static double AUDIENCE_CELL_Y = 54.25;

    // TODO(12): Drive across the middle. Raise it if the turret flips between cells (or between shoot and pass) too eagerly.
    /**
     * Which half of the field the robot is on (far or audience) picks the cell it
     * aims at, or with an up cell picked, shoot or pass. Within this many inches
     * of the center line the half stays what it was, so it doesn't flip back and
     * forth while driving along the middle.
     */
    public static double CELL_SWITCH_BAND = 6;

    // TODO(7): Main TeleOp, pick an up cell (driver 2 LB/RB) and fire from the other half: pick where passes should land, set X and Y, and check real passes land there on both halves and none leaves the field (G405).
    /**
     * Where a pass lands, written for red's far half: in front of the middle of
     * our LOADING ZONE (y 94.66 to 117.35, official field CAD). A pass goes to the
     * target on the robot's own half (passTarget), and the audience half's is this
     * one mirrored across the center line. Placeholder.
     */
    public static double PASS_TARGET_RED_X = 24;
    public static double PASS_TARGET_RED_Y = 106;

    // TODO(8): CAD says 203 mm. Check with a tape: intake flat on a wall, wall to turret center. Is the stowed flower intake further out?
    /** Robot center to the intake surface that touches a wall. */
    public static double CENTER_TO_INTAKE_WALL = 7.99;
    // TODO(8): CAD outer frame rails say 149 mm (298 wide, not 280). Check with a tape.
    /** Robot center to the side surface that touches a wall. */
    public static double CENTER_TO_SIDE_WALL = 5.87;

    private Field() {}

    public static Pose cellTarget(Alliance alliance, Cell cell) {
        // Only x flips for blue: the cells are fixed geometry, so only which HIVE is "ours" mirrors.
        // Each HIVE has a far and an audience cell at these same y values. That relies on
        // FAR_CELL_Y + AUDIENCE_CELL_Y == SIZE, so re-check the pair if you re-measure either one.
        double x = alliance == Alliance.RED ? RED_HIVE_X : SIZE - RED_HIVE_X;
        double y = cell == Cell.FAR ? FAR_CELL_Y : AUDIENCE_CELL_Y;
        return new Pose(x, y);
    }

    /**
     * The robot's half of the field, named by the cell on that half (FAR or
     * AUDIENCE). Within CELL_SWITCH_BAND of the center line it keeps current.
     */
    public static Cell cellForSide(double robotY, Cell current) {
        double center = SIZE / 2;
        if (robotY > center + CELL_SWITCH_BAND) return Cell.FAR;
        if (robotY < center - CELL_SWITCH_BAND) return Cell.AUDIENCE;
        return current;
    }

    /**
     * Our HIVE's cell on the driver's left (or right), as the driver sees it from
     * the ALLIANCE AREA. Red's left is the far side, blue's the audience side.
     */
    public static Cell cellFromDriver(Alliance alliance, boolean left) {
        if (alliance == Alliance.RED) return left ? Cell.FAR : Cell.AUDIENCE;
        return left ? Cell.AUDIENCE : Cell.FAR;
    }

    /**
     * Where a pass from this half of the field lands: always on the robot's own
     * half, near its own alliance wall. Like the cells, only x flips for blue.
     * (That is the same as spinning red's target for the other half with
     * Alliance.fromRed.)
     */
    public static Pose passTarget(Alliance alliance, Cell robotHalf) {
        double x = alliance == Alliance.RED ? PASS_TARGET_RED_X : SIZE - PASS_TARGET_RED_X;
        double y = robotHalf == Cell.FAR ? PASS_TARGET_RED_Y : SIZE - PASS_TARGET_RED_Y;
        return new Pose(x, y);
    }

    // TODO(8): Relocalize in all 4 corners and on both side walls (driver 2: D-pad down/up, Cross, Triangle, D-pad left, Circle), drive to a taped spot, and check the pose.
    public static Pose cornerPose(Alliance alliance, Corner corner) {
        double intake = CENTER_TO_INTAKE_WALL;
        double side = CENTER_TO_SIDE_WALL;
        // Written for red: the driver looks along +x, so the driver's left is +y (the far wall).
        Pose red;
        switch (corner) {
            case NEAR_LEFT:
                red = new Pose(intake, SIZE - side, Math.PI);
                break;
            case NEAR_RIGHT:
                red = new Pose(intake, side, Math.PI);
                break;
            case FAR_LEFT:
                red = new Pose(SIZE - intake, SIZE - side, 0);
                break;
            case FAR_RIGHT:
            default:
                red = new Pose(SIZE - intake, side, 0);
                break;
        }
        return alliance.fromRed(red);
    }

    /**
     * The heading, in radians, with the intake flat against the left (or right)
     * side wall, as the driver sees it from the ALLIANCE AREA. Red's left side wall
     * is the far wall, blue's the audience wall.
     */
    public static double sideWallHeading(Alliance alliance, boolean left) {
        double turn = left ? Math.PI / 2 : -Math.PI / 2;
        return Angles.wrapRadians(alliance.driverHeading() + turn);
    }
}
