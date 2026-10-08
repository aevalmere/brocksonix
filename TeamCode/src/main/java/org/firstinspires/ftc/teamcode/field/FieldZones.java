package org.firstinspires.ftc.teamcode.field;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.agateflow.Zone;

import java.util.ArrayList;
import java.util.List;

/**
 * The fixed things on the BIOBUZZ field AgateFlow must drive around, as
 * keep-out zones. Field coordinates (see Field). The walls are always added by
 * FieldMap, so they aren't here.
 *
 * Give a zone the real outline of the thing, from the tiles up to the robot's
 * top. AgateFlow adds the robot's size and a margin itself
 * (AgateFlow.ROBOT_RADIUS, SAFETY_MARGIN).
 *
 * What a robot can hit (Competition Manual TU03, Section 9, checked against the
 * official field CAD):
 * <ul>
 * <li><b>HIVE frame</b> (9.6.1, Figures 9-8 and 9-10): an A-frame at each end
 *     (red and blue), joined by a crossbar at the top. Each A-frame stands on a
 *     foot bar on the tiles that runs along y, and its two legs lean in, toward
 *     each other and toward the field's middle, up to the HIVE pivot 43.95 in.
 *     up. So the higher up, the further in the legs are. Each A-frame's zone is
 *     its foot bar plus its legs from the tiles up to the robot's top, filled
 *     in.</li>
 * <li><b>HIVES</b> never come lower than 30.6 in. above the tiles (Figure
 *     9-10), and our robot is 12.74 in. tall. So tipping changes nothing on the
 *     floor, and the robot fits under the HIVES, between the A-frames. The
 *     crossbar and the banners are above 34 in., and the bars that hold the
 *     frame down are under the tiles.</li>
 * <li><b>FLOWERS</b> (9.7, Figure 9-12): 4 on the walls, 4.9 in. deep.</li>
 * <li>The GARDENS and LOADING ZONES are only tape.</li>
 * </ul>
 *
 * Not used by any OpMode yet.
 */
@Configurable
public final class FieldZones {
    /** The robot's height, inches (CAD). The HIVE frame zones cover the legs from the tiles up to here. */
    public static double ROBOT_HEIGHT = 12.73597;

    // ---- HIVE frame. Centered on the field, so both ends use the same numbers. ----
    /** Across the outside of the two foot bars, along x (red wall to blue wall), inches. 9.6.1. */
    public static double FRAME_WIDTH = 49.46;
    /** Length of each foot bar, along y, inches. 9.6.1. */
    public static double FRAME_DEPTH = 38.95;
    /** Width of one foot bar along x, inches. Official field CAD (1.98); Pedro Visualizer draws about 1.9. */
    public static double FOOT_BAR_WIDTH = 2.0;
    /** Height of the HIVE pivot, inches. 9.6.1. Each end's legs lean in to the pivot, on its HIVE's center line. */
    public static double PIVOT_HEIGHT = 43.95;
    /** How thick a leg looks from above, inches. It's a 1 in. square tube, and it leans (official field CAD). */
    public static double LEG_THICKNESS = 1.2;

    // ---- FLOWERS. Red's are written here; blue's are the same spun 180°. ----
    /** Red's FLOWERS are centered here: y on the red wall, x on the far wall, inches. Pedro Visualizer (official field CAD: 47.4). */
    public static double FLOWER_ALONG_WALL = 47.3;
    /** Wall to a FLOWER's front face, inches. Front face to ring center 2.40 (Figure 9-12), ring center to wall 2.5 (official field CAD). */
    public static double FLOWER_DEPTH = 4.9;
    /**
     * Width of a FLOWER along its wall, inches. The rings are 5.9 wide (Figure 9-12,
     * Pedro Visualizer). The bracket on top of the wall is 6.7 wide and 11.4 in. up,
     * under the robot's top (Figure 9-17, official field CAD), so the zone uses that.
     */
    public static double FLOWER_WIDTH = 6.7;

    private FieldZones() {}

    /** Built from the numbers above every time, so a Panels edit applies to the next plan. */
    public static List<Zone> zones() {
        List<Zone> zones = new ArrayList<>();
        zones.add(frameEnd("HIVE frame, red end", Alliance.RED));
        zones.add(frameEnd("HIVE frame, blue end", Alliance.BLUE));
        zones.add(redWallFlower("FLOWER, red wall"));
        zones.add(farWallFlower("FLOWER, far wall"));
        zones.add(redWallFlower("FLOWER, blue wall").forAlliance(Alliance.BLUE));
        zones.add(farWallFlower("FLOWER, audience wall").forAlliance(Alliance.BLUE));
        return zones;
    }

    /**
     * Extra keep-out for AUTO only: the other alliance's half of the field.
     * During AUTO, tile columns A-C (x 0 to 70.75) are red's side and D-F are
     * blue's (G402, Figure 9-5). Driving onto the other side "is a risky gameplay
     * strategy that may be seen as STRATEGIC" (a MAJOR FOUL for disrupting the
     * other alliance's AUTO), so an auto adds this to stay on its own side:
     * {@code agateFlow.map().setZones("auto", FieldZones.autoZones(alliance))}.
     */
    public static List<Zone> autoZones(Alliance alliance) {
        List<Zone> zones = new ArrayList<>();
        double center = Field.SIZE / 2;
        zones.add(Zone.rectangle("other alliance's side (AUTO, G402)", center, 0, Field.SIZE, Field.SIZE)
                .forAlliance(alliance));
        return zones;
    }

    /**
     * One end of the HIVE frame, written for the red end and spun for the blue one.
     *
     * The legs' outside edges run straight from the corners of the frame's base
     * (FRAME_WIDTH x FRAME_DEPTH) to the pivot, PIVOT_HEIGHT up on the HIVE's center
     * line. For our 12.74 in. robot, measured from the field center:
     * <pre>
     * up        = 12.74 / 43.95                      = 0.29 of the way up
     * pivot     = 70.75 - 58.0                       = 12.75 along x
     * legsOut   = 24.73 + (12.75 - 24.73) * 0.29     = 21.26 along x
     * legsIn    = 21.26 - 1.2                        = 20.06 along x
     * legSpread = 19.475 * (1 - 0.29)                = 13.83 along y
     * </pre>
     * So the red end's zone is x 46.02 to 50.69 and y 51.28 to 90.23, narrowing to
     * y 56.92 to 84.58 at its inside edge. This is within 0.25 in. of the official
     * field CAD, and Pedro Visualizer draws the foot bar within 0.3 in. of it.
     */
    private static Zone frameEnd(String name, Alliance end) {
        double center = Field.SIZE / 2;
        double up = ROBOT_HEIGHT / PIVOT_HEIGHT;
        double pivot = center - Field.RED_HIVE_X;
        double legsOut = FRAME_WIDTH / 2 + (pivot - FRAME_WIDTH / 2) * up;
        double legsIn = legsOut - LEG_THICKNESS;
        double legSpread = FRAME_DEPTH / 2 * (1 - up);

        double outside = center - FRAME_WIDTH / 2;
        double inside = outside + FOOT_BAR_WIDTH;
        double legs = center - legsIn;
        double barFront = center - FRAME_DEPTH / 2, barBack = center + FRAME_DEPTH / 2;
        double legFront = center - legSpread, legBack = center + legSpread;
        // The foot bar, and the legs where they reach the robot's top. The legs are straight, so
        // lower down they are on the lines between these points.
        return Zone.polygon(name,
                outside, barFront, inside, barFront, legs, legFront,
                legs, legBack, inside, barBack, outside, barBack).forAlliance(end);
    }

    /** Red's FLOWER on the red wall. */
    private static Zone redWallFlower(String name) {
        double half = FLOWER_WIDTH / 2;
        return Zone.rectangle(name, 0, FLOWER_ALONG_WALL - half, FLOWER_DEPTH, FLOWER_ALONG_WALL + half);
    }

    /** Red's FLOWER on the far wall. */
    private static Zone farWallFlower(String name) {
        double half = FLOWER_WIDTH / 2;
        return Zone.rectangle(name, FLOWER_ALONG_WALL - half, Field.SIZE - FLOWER_DEPTH, FLOWER_ALONG_WALL + half, Field.SIZE);
    }
}
