package org.firstinspires.ftc.teamcode.zenith;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.robot.commands.RobotCommands;
import org.firstinspires.ftc.teamcode.util.Angles;
import org.horizon36596.zenith.AllianceFrame;
import org.horizon36596.zenith.ivy.NamedCommands;
import org.horizon36596.zenith.ivy.ZenithRobot;
import org.junit.After;
import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.List;

/**
 * The Zenith bridge without hardware: the names an auto file can use are real
 * RobotCommands methods, and a pose from a file lands in the right place on our
 * field for both alliances, flipped once for blue.
 */
public class TeamZenithRobotTest {
    private static final double EPS = 1e-9;

    @After
    public void clearRegistry() {
        // NamedCommands is static: leave it empty for the next test.
        NamedCommands.reset();
    }

    private static void assertPose(String what, double x, double y, double heading, Pose pose) {
        assertEquals(what + " x", x, pose.x(), EPS);
        assertEquals(what + " y", y, pose.y(), EPS);
        // Compared the short way around, so -π and π are the same heading.
        assertEquals(what + " heading", 0, Angles.wrapRadians(pose.heading() - heading), EPS);
    }

    @Test
    public void everyCommandNameIsARobotCommandsMethod() {
        NamedCommands.reset();
        // The factories only run when a routine is built, so no Robot is needed to register them.
        new TeamZenithRobot().registerCommands(null);
        List<String> names = NamedCommands.commandNames();
        assertFalse("nothing registered", names.isEmpty());
        for (String name : names) {
            boolean found = false;
            for (Method method : RobotCommands.class.getMethods()) {
                if (method.getName().equals(name) && Command.class.isAssignableFrom(method.getReturnType())
                        && !Modifier.isStatic(method.getModifiers())) {
                    found = true;
                }
            }
            assertTrue(name + " is not a RobotCommands method that returns a Command", found);
        }
    }

    @Test
    public void theNamesAutoFilesUse() {
        NamedCommands.reset();
        new TeamZenithRobot().registerCommands(null);
        // robot.json and the auto files name these; renaming one breaks every auto that uses it.
        assertEquals(Arrays.asList("flowerDown", "flowerUp", "shoot", "shootAll", "shootAllWithRetry",
                "shootContinuous", "unjam", "waitUntilFull"), NamedCommands.commandNames());
        assertEquals(Arrays.asList("canFire", "storageEmpty", "storageFull"), NamedCommands.conditionNames());
    }

    @Test
    public void filePosesMoveOntoOurField() {
        double center = ZenithFrame.CENTER;
        assertPose("center", center, center, 0, ZenithFrame.toPedro(new Pose(0, 0, 0)));
        // Zenith's starter start pose: 12 in. left of center, 63 in. toward the audience, facing away from it.
        assertPose("start", center - 12, center - 63, Math.PI / 2, ZenithFrame.toPedro(new Pose(-12, -63, Math.PI / 2)));
    }

    @Test
    public void blueIsRedSpunAroundTheCenter() {
        TeamZenithRobot robot = new TeamZenithRobot();
        Pose red = ZenithFrame.toPedro(new Pose(-12, -63, Math.PI / 2));
        Pose blue = robot.mirror(red);
        assertPose("blue", Field.SIZE - red.x(), Field.SIZE - red.y(), red.heading() + Math.PI, blue);
        // The same place Zenith shows for blue: its own point mirror (-x, -y, h + π), then moved onto our field.
        assertPose("Zenith's blue", 12 + ZenithFrame.CENTER, 63 + ZenithFrame.CENTER, -Math.PI / 2, blue);
        // Flipping twice is back on red, so a pose flipped once more than it should be is easy to spot.
        assertPose("twice", red.x(), red.y(), red.heading(), robot.mirror(blue));
    }

    @Test
    public void zenithAcceptsBothFrames() {
        TeamZenithRobot robot = new TeamZenithRobot();
        // Zenith checks at init that the frame turns headings the way it moves poses. Both must pass,
        // and neither may reverse turns (it's a spin, not a reflection).
        AllianceFrame red = AllianceFrame.of(ZenithFrame::toPedro);
        AllianceFrame blue = AllianceFrame.of(p -> robot.mirror(ZenithFrame.toPedro(p)));
        assertFalse(red.reversesTurns());
        assertFalse(blue.reversesTurns());
        assertEquals(0.5, red.heading(0.5), EPS);
        assertEquals(0, Angles.wrapRadians(blue.heading(0.5) - (0.5 + Math.PI)), EPS);
    }

    /**
     * What zenith deploy writes for each auto file once zenith.json names ZenithAuto as
     * deploy.opModeBase and TeamZenithRobot as deploy.robotClass. Only compiled, never run.
     */
    @SuppressWarnings("unused")
    static final class GeneratedAutoShape extends ZenithAuto {
        @Override
        protected String autoName() {
            return "first-auto";
        }

        @Override
        protected ZenithRobot createRobot() {
            return new TeamZenithRobot();
        }
    }
}
