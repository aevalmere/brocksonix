package org.firstinspires.ftc.teamcode.field;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.util.Angles;
import org.junit.Test;

/**
 * Alliance and Field: blue is red spun 180° around the field center, the cells
 * mirror only in x, corner poses touch the walls, "left" is the same side for
 * corners, side walls and cells, the robot's half has a band in the middle, and
 * the pass target sits on the robot's own half. The numbers come from
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

    private static void assertHeading(String what, double expected, double actual) {
        assertEquals(what, 0, Angles.wrapRadians(actual - expected), EPS);
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
        // Red: the driver looks along +x, so left is +y. Near corners have the intake on the red
        // wall (facing the driver, -x), far corners on the blue wall (facing away, +x).
        assertPose("red near left", intake, far - side, Math.PI, Field.cornerPose(Alliance.RED, Corner.NEAR_LEFT));
        assertPose("red near right", intake, side, Math.PI, Field.cornerPose(Alliance.RED, Corner.NEAR_RIGHT));
        assertPose("red far left", far - intake, far - side, 0, Field.cornerPose(Alliance.RED, Corner.FAR_LEFT));
        assertPose("red far right", far - intake, side, 0, Field.cornerPose(Alliance.RED, Corner.FAR_RIGHT));
        // Blue: the driver looks along -x, so left is -y (the audience wall).
        assertPose("blue near left", far - intake, side, 0, Field.cornerPose(Alliance.BLUE, Corner.NEAR_LEFT));
        assertPose("blue near right", far - intake, far - side, 0, Field.cornerPose(Alliance.BLUE, Corner.NEAR_RIGHT));
        assertPose("blue far left", intake, side, Math.PI, Field.cornerPose(Alliance.BLUE, Corner.FAR_LEFT));
        assertPose("blue far right", intake, far - side, Math.PI, Field.cornerPose(Alliance.BLUE, Corner.FAR_RIGHT));
    }

    @Test
    public void sideWallHeadingsFaceTheDriversLeftAndRight() {
        // Red's left side wall is the far wall (+y), blue's the audience wall (-y).
        assertHeading("red left", 0.5 * Math.PI, Field.sideWallHeading(Alliance.RED, true));
        assertHeading("red right", 1.5 * Math.PI, Field.sideWallHeading(Alliance.RED, false));
        assertHeading("blue left", 1.5 * Math.PI, Field.sideWallHeading(Alliance.BLUE, true));
        assertHeading("blue right", 0.5 * Math.PI, Field.sideWallHeading(Alliance.BLUE, false));
    }

    @Test
    public void leftMeansTheSameSideForCornersWallsAndCells() {
        double center = Field.SIZE / 2;
        for (Alliance alliance : Alliance.values()) {
            // The left corners, the left side wall heading and the left cell all point the same way in y.
            double leftCornerY = Field.cornerPose(alliance, Corner.NEAR_LEFT).y() - center;
            double farLeftCornerY = Field.cornerPose(alliance, Corner.FAR_LEFT).y() - center;
            double leftWallY = Math.sin(Field.sideWallHeading(alliance, true));
            double leftCellY = Field.cellTarget(alliance, Field.cellFromDriver(alliance, true)).y() - center;
            assertEquals(alliance + " far left corner", Math.signum(leftCornerY), Math.signum(farLeftCornerY), EPS);
            assertEquals(alliance + " left wall", Math.signum(leftCornerY), Math.signum(leftWallY), EPS);
            assertEquals(alliance + " left cell", Math.signum(leftCornerY), Math.signum(leftCellY), EPS);
        }
    }

    @Test
    public void bumpersPickTheCellOnThatSideOfTheDriver() {
        assertEquals(Cell.FAR, Field.cellFromDriver(Alliance.RED, true));
        assertEquals(Cell.AUDIENCE, Field.cellFromDriver(Alliance.RED, false));
        assertEquals(Cell.AUDIENCE, Field.cellFromDriver(Alliance.BLUE, true));
        assertEquals(Cell.FAR, Field.cellFromDriver(Alliance.BLUE, false));
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
    public void thePassTargetIsOnTheRobotsHalfNearItsOwnWall() {
        double x = Field.PASS_TARGET_RED_X;
        double y = Field.PASS_TARGET_RED_Y;
        double far = Field.SIZE;
        assertPose("red, far half", x, y, 0, Field.passTarget(Alliance.RED, Cell.FAR));
        assertPose("red, audience half", x, far - y, 0, Field.passTarget(Alliance.RED, Cell.AUDIENCE));
        assertPose("blue, far half", far - x, y, 0, Field.passTarget(Alliance.BLUE, Cell.FAR));
        assertPose("blue, audience half", far - x, far - y, 0, Field.passTarget(Alliance.BLUE, Cell.AUDIENCE));
        // Blue's is red's target for the other half, spun 180°.
        Pose spun = Alliance.BLUE.fromRed(Field.passTarget(Alliance.RED, Cell.AUDIENCE));
        Pose blueFar = Field.passTarget(Alliance.BLUE, Cell.FAR);
        assertEquals(spun.x(), blueFar.x(), EPS);
        assertEquals(spun.y(), blueFar.y(), EPS);
        // Always on the robot's own half, and on its own alliance's side of the center.
        double center = far / 2;
        for (Alliance alliance : Alliance.values()) {
            assertTrue(alliance + " far", Field.passTarget(alliance, Cell.FAR).y() > center);
            assertTrue(alliance + " audience", Field.passTarget(alliance, Cell.AUDIENCE).y() < center);
            double ownSide = alliance == Alliance.RED ? -1 : 1;
            assertEquals(alliance + " near its wall", ownSide, Math.signum(Field.passTarget(alliance, Cell.FAR).x() - center), EPS);
        }
    }
}
