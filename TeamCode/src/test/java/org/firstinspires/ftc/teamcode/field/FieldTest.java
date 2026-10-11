package org.firstinspires.ftc.teamcode.field;

import static org.junit.Assert.assertEquals;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.util.Angles;
import org.junit.Test;

/**
 * Alliance and Field: blue is red spun 180° around the field center, the cells
 * mirror only in x, corner poses touch the walls, and the target cell follows the
 * robot's half of the field with a band in the middle. The numbers come from
 * Field's own values, so measuring them again doesn't break these.
 */
public class FieldTest {
    private static final double EPS = 1e-9;

    private static void assertPose(String what, double x, double y, double heading, Pose pose) {
        assertEquals(what + " x", x, pose.x(), EPS);
        assertEquals(what + " y", y, pose.y(), EPS);
        // Compared the short way around, so -π and π are the same heading.
        assertEquals(what + " heading", 0, Angles.wrapRadians(pose.heading() - heading), EPS);
    }

    @Test
    public void redPosesStayAsWritten() {
        Pose red = new Pose(20, 30, 0.5);
        assertPose("red", 20, 30, 0.5, Alliance.RED.fromRed(red));
    }

    @Test
    public void bluePosesAreRedSpunAroundTheCenter() {
        Pose blue = Alliance.BLUE.fromRed(new Pose(20, 30, 0.5));
        assertPose("blue", Field.SIZE - 20, Field.SIZE - 30, 0.5 + Math.PI, blue);
        // Spinning twice is back where it started.
        assertPose("twice", 20, 30, 0.5, Alliance.BLUE.fromRed(blue));
        // The center stays put.
        double center = Field.SIZE / 2;
        assertPose("center", center, center, Math.PI, Alliance.BLUE.fromRed(new Pose(center, center, 0)));
    }

    @Test
    public void otherAllianceAndDriverHeading() {
        assertEquals(Alliance.BLUE, Alliance.RED.other());
        assertEquals(Alliance.RED, Alliance.BLUE.other());
        assertEquals(0, Alliance.RED.driverHeading(), EPS);
        assertEquals(Math.PI, Alliance.BLUE.driverHeading(), EPS);
    }

    @Test
    public void cellsMirrorOnlyInX() {
        Pose redFar = Field.cellTarget(Alliance.RED, Cell.FAR);
        Pose blueFar = Field.cellTarget(Alliance.BLUE, Cell.FAR);
        assertEquals(Field.RED_HIVE_X, redFar.x(), EPS);
        assertEquals(Field.FAR_CELL_Y, redFar.y(), EPS);
        assertEquals(Field.SIZE - Field.RED_HIVE_X, blueFar.x(), EPS);
        assertEquals(Field.FAR_CELL_Y, blueFar.y(), EPS);
        assertEquals(Field.AUDIENCE_CELL_Y, Field.cellTarget(Alliance.BLUE, Cell.AUDIENCE).y(), EPS);
    }

    @Test
    public void cornerPosesTouchTheWalls() {
        double intake = Field.CENTER_TO_INTAKE_WALL;
        double side = Field.CENTER_TO_SIDE_WALL;
        double far = Field.SIZE;
        // Red: intake on the red wall (facing -x), side on the audience or far wall.
        assertPose("right, intake to driver", intake, side, Math.PI,
                Field.cornerPose(Alliance.RED, Corner.RIGHT_INTAKE_TO_DRIVER));
        assertPose("left, intake to driver", intake, far - side, Math.PI,
                Field.cornerPose(Alliance.RED, Corner.LEFT_INTAKE_TO_DRIVER));
        // Intake on the audience or far wall, side on the red wall.
        assertPose("right, intake to side wall", side, intake, 1.5 * Math.PI,
                Field.cornerPose(Alliance.RED, Corner.RIGHT_INTAKE_TO_SIDE_WALL));
        assertPose("left, intake to side wall", side, far - intake, 0.5 * Math.PI,
                Field.cornerPose(Alliance.RED, Corner.LEFT_INTAKE_TO_SIDE_WALL));
        // Blue: the same corners, spun to the blue wall.
        assertPose("blue right, intake to driver", far - intake, far - side, 0,
                Field.cornerPose(Alliance.BLUE, Corner.RIGHT_INTAKE_TO_DRIVER));
    }

    @Test
    public void theCellFollowsTheRobotsHalfWithABandInTheMiddle() {
        double center = Field.SIZE / 2;
        double band = Field.CELL_SWITCH_BAND;
        assertEquals(Cell.FAR, Field.cellForSide(center + band + 1, Cell.AUDIENCE));
        assertEquals(Cell.AUDIENCE, Field.cellForSide(center - band - 1, Cell.FAR));
        // Inside the band it keeps the cell it had.
        assertEquals(Cell.FAR, Field.cellForSide(center, Cell.FAR));
        assertEquals(Cell.AUDIENCE, Field.cellForSide(center + band - 1, Cell.AUDIENCE));
    }

    @Test
    public void thePassTargetIsWrittenForRed() {
        Pose red = Field.passTarget(Alliance.RED);
        Pose blue = Field.passTarget(Alliance.BLUE);
        assertEquals(Field.PASS_TARGET_RED_X, red.x(), EPS);
        assertEquals(Field.PASS_TARGET_RED_Y, red.y(), EPS);
        assertEquals(Field.SIZE - Field.PASS_TARGET_RED_X, blue.x(), EPS);
        assertEquals(Field.SIZE - Field.PASS_TARGET_RED_Y, blue.y(), EPS);
    }
}
