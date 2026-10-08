package org.firstinspires.ftc.teamcode.field;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

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
 * Positions follow Pedro Visualizer, and sizes come from Competition Manual
 * Section 9 (TU03). Both were checked against the official field CAD (141.06
 * wide; the manual says about 144). See TELEOP_PLAN.md, "Field coordinates",
 * for where each number came from.
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

    // TODO(12): Drive across the middle. Raise it if the turret flips between cells too eagerly.
    /**
     * The robot aims at the cell on its own half (far or audience). Within this
     * many inches of the center line it keeps the current cell, so it doesn't
     * flip back and forth while driving along the middle.
     */
    public static double CELL_SWITCH_BAND = 6;

    /**
     * Where a pass lands, written for red: in front of the middle of our LOADING
     * ZONE (y 94.5 to 117.6). Placeholder: passing is on hold until the variable hood.
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

    public static Cell cellForSide(double robotY, Cell current) {
        double center = SIZE / 2;
        if (robotY > center + CELL_SWITCH_BAND) return Cell.FAR;
        if (robotY < center - CELL_SWITCH_BAND) return Cell.AUDIENCE;
        return current;
    }

    public static Pose passTarget(Alliance alliance) {
        return alliance.fromRed(new Pose(PASS_TARGET_RED_X, PASS_TARGET_RED_Y));
    }

    // TODO(8): Relocalize in all 4 corners, drive to a taped spot, and check the pose.
    public static Pose cornerPose(Alliance alliance, Corner corner) {
        double intake = CENTER_TO_INTAKE_WALL;
        double side = CENTER_TO_SIDE_WALL;
        Pose red;
        switch (corner) {
            case RIGHT_INTAKE_TO_DRIVER:
                red = new Pose(intake, side, Math.PI);
                break;
            case LEFT_INTAKE_TO_DRIVER:
                red = new Pose(intake, SIZE - side, Math.PI);
                break;
            case RIGHT_INTAKE_TO_SIDE_WALL:
                red = new Pose(side, intake, 1.5 * Math.PI);
                break;
            case LEFT_INTAKE_TO_SIDE_WALL:
            default:
                red = new Pose(side, SIZE - intake, 0.5 * Math.PI);
                break;
        }
        return alliance.fromRed(red);
    }
}
