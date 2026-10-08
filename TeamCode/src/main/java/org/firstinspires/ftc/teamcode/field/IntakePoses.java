package org.firstinspires.ftc.teamcode.field;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

/**
 * Where the robot picks up POLLEN on our side of the field during AUTO: our two
 * FLOWERS and our GARDEN. Each spot is a pose for the robot's center (the turret
 * center), facing what it picks up, plus the direction to drive in at the end
 * (for Route.approach). The robot always drives in forward, intake first.
 *
 * Written for red, like every pose: Route flips them for blue (use
 * Alliance.fromRed anywhere else).
 *
 * <pre>
 * new Route()
 *         .to(IntakePoses.flowerPose(Flower.ALLIANCE_WALL))
 *         .approach(IntakePoses.flowerTravelDegrees(Flower.ALLIANCE_WALL), IntakePoses.FLOWER_APPROACH);
 * new Route()                                  // the GARDEN: line up, then run along the wall
 *         .to(IntakePoses.gardenStartPose())
 *         .to(IntakePoses.gardenEndPose()).allowContact().maxSpeed(20);
 * </pre>
 *
 * Not used by any OpMode yet.
 */
@Configurable
public final class IntakePoses {
    // ---- FLOWER: POLLEN only come out of the Retrieval Opening at the bottom, 3.55 in. tall ----
    // ---- and 3.57 in. deep (9.7, G418). The robot drives in with the flower intake down.     ----
    // TODO(14): Measure on the robot, flower intake down: turret center to where it touches the tiles (ROBOT.md says 275.3 mm).
    /** Turret center to where the lowered flower intake touches the tiles, inches. */
    public static double FLOWER_INTAKE_REACH = 10.84;
    // TODO(14): Drive into a real FLOWER. Raise this if the flower intake doesn't reach the POLLEN, lower it if the robot's front hits the FLOWER.
    /**
     * How far into the Retrieval Opening, past the FLOWER's front face, the flower
     * intake touches down, inches. 2.40 is right under the bottom POLLEN (Figure
     * 9-12: front face to ring center). The opening ends at 3.57.
     */
    public static double FLOWER_REACH_INTO_OPENING = 2.4;
    // TODO(14): Shorten it if there's no room to line up, lengthen it if the flower intake catches the FLOWER's side.
    /** Drive straight at the FLOWER for this many inches at the end, so the flower intake goes in square. */
    public static double FLOWER_APPROACH = 12;

    // ---- GARDEN: 4 POLLEN in a line from our corner, touching the audience wall (10.3.1). ----
    // ---- They go in through the intake at the robot's front (ROBOT.md), flower intake up. ----
    // ---- The intake is as wide as the chassis, so it reaches POLLEN against the wall.      ----
    /** Red wall to the far side of the 4th POLLEN, inches: 4 touching POLLEN, 2.8 in. each (9.8). Pedro Visualizer and the official field CAD show 11.3. */
    public static double GARDEN_LINE_LENGTH = 11.2;
    // TODO(14): Raise it if the robot isn't lined up yet when the intake reaches the line.
    /** At the start, the gap between the intake and the 4th POLLEN, inches. */
    public static double GARDEN_RUN_UP = 4;
    // TODO(14): Lower it if the POLLEN in the corner is left behind.
    /** At the end, the gap between the intake and the red wall, inches. */
    public static double GARDEN_END_GAP = 1;
    // TODO(14): Raise it if the robot rubs the audience wall on the GARDEN run.
    /** Gap between the robot's side and the audience wall while driving the line, inches. */
    public static double GARDEN_WALL_GAP = 0.5;

    private IntakePoses() {}

    /**
     * Where the robot's center must be to pull POLLEN out of this FLOWER, facing it.
     * Wall to turret center: FLOWER_DEPTH - FLOWER_REACH_INTO_OPENING + FLOWER_INTAKE_REACH
     * = 4.9 - 2.4 + 10.84 = 13.34 in. Then the intake's front (Field.CENTER_TO_INTAKE_WALL,
     * 7.99) is 13.34 - 7.99 = 5.35 in. from the wall, just short of the FLOWER's front face at 4.9.
     * For red that's (13.34, 47.3) facing the red wall, and (47.3, 141.5 - 13.34 = 128.16)
     * facing the far wall.
     */
    public static Pose flowerPose(Flower flower) {
        double fromWall = FieldZones.FLOWER_DEPTH - FLOWER_REACH_INTO_OPENING + FLOWER_INTAKE_REACH;
        double along = FieldZones.FLOWER_ALONG_WALL;
        switch (flower) {
            case ALLIANCE_WALL:
                return new Pose(fromWall, along, Math.PI);
            case LEFT_WALL:
            default:
                return new Pose(along, Field.SIZE - fromWall, 0.5 * Math.PI);
        }
    }

    /** Direction to drive in for the last FLOWER_APPROACH inches: straight at the wall, in degrees. */
    public static double flowerTravelDegrees(Flower flower) {
        return flower == Flower.ALLIANCE_WALL ? 180 : 90;
    }

    /*
     * The GARDEN run drives along the line toward the corner, since there's no room to start on
     * the corner side. The whole run hugs the audience wall, so its waypoint needs
     * Route.allowContact(), or AgateFlow keeps the robot a full margin off the wall.
     */

    /**
     * Start of the GARDEN run: facing the red wall, the intake GARDEN_RUN_UP short of the 4th POLLEN.
     * x = 11.2 + 4 + 7.99 = 23.19, y = 5.87 + 0.5 = 6.37.
     */
    public static Pose gardenStartPose() {
        return new Pose(GARDEN_LINE_LENGTH + GARDEN_RUN_UP + Field.CENTER_TO_INTAKE_WALL, gardenLaneY(), Math.PI);
    }

    /** End of the GARDEN run: the intake GARDEN_END_GAP from the red wall, all 4 POLLEN passed. x = 7.99 + 1 = 8.99. */
    public static Pose gardenEndPose() {
        return new Pose(Field.CENTER_TO_INTAKE_WALL + GARDEN_END_GAP, gardenLaneY(), Math.PI);
    }

    /** Direction to drive from gardenStartPose() to gardenEndPose(): toward the red wall, in degrees. */
    public static double gardenTravelDegrees() {
        return 180;
    }

    private static double gardenLaneY() {
        return Field.CENTER_TO_SIDE_WALL + GARDEN_WALL_GAP;
    }
}
