package org.firstinspires.ftc.teamcode.field;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

/**
 * Where things are on the BIOBUZZ field, in inches.
 *
 * Standing where the audience stands: (0, 0) is the near-left corner (red
 * alliance wall and audience wall), +x points toward the blue wall, +y away
 * from the audience. Heading 0 faces +x and counter-clockwise is positive.
 *
 * Values come from Competition Manual Section 9 (TU02). See TELEOP_PLAN.md,
 * "Field coordinates", for where each number came from.
 */
@Configurable
public final class Field {
    public static final double SIZE = 144;

    // TODO(8): On a real field, measure from the red wall to the red HIVE's center line.
    /** Red HIVE center line. Blue's is mirrored across the field center. */
    public static double RED_HIVE_X = 59.25;
    // TODO(8): On a real field, measure from the audience wall to the upward cell's opening center, once with each cell up.
    /** Center of the upward cell when the far cell is up. Computed from Figure 9-10. */
    public static double FAR_CELL_Y = 85.4;
    /** Center of the upward cell when the audience cell is up. Computed from Figure 9-10. */
    public static double AUDIENCE_CELL_Y = 58.6;

    // TODO(8): Drive across the middle. Raise it if the turret flips between cells too eagerly.
    /**
     * The robot aims at the cell on its own half (far or audience). Within this
     * many inches of the center line it keeps the current cell, so it doesn't
     * flip back and forth while driving along the middle.
     */
    public static double CELL_SWITCH_BAND = 6;

    /** Where a pass lands, written for red. Placeholder: passing is on hold until the variable hood. */
    public static double PASS_TARGET_RED_X = 24;
    public static double PASS_TARGET_RED_Y = 108;

    // TODO(8): CAD says 203 mm. Check with a tape: intake flat on a wall, wall to turret center. Is the stowed flower intake further out?
    /** Robot center to the intake surface that touches a wall. */
    public static double CENTER_TO_INTAKE_WALL = 7.99;
    // TODO(8): CAD outer frame rails say 149 mm (298 wide, not 280). Check with a tape.
    /** Robot center to the side surface that touches a wall. */
    public static double CENTER_TO_SIDE_WALL = 5.87;

    private Field() {}

    public static Pose cellTarget(Alliance alliance, Cell cell) {
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
