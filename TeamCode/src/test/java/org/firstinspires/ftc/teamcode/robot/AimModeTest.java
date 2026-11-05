package org.firstinspires.ftc.teamcode.robot;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.field.Cell;
import org.junit.Test;

/**
 * Robot.decideAimMode: shoot on the up cell's half or with no up cell picked
 * (auto never picks one), pass from the other half, and nothing there while
 * passing is off.
 */
public class AimModeTest {

    @Test
    public void noUpCellAlwaysShootsAtTheOwnHalf() {
        for (Cell half : Cell.values()) {
            assertEquals(FireControl.Mode.SHOOT, Robot.decideAimMode(null, half, true));
            assertEquals(FireControl.Mode.SHOOT, Robot.decideAimMode(null, half, false));
        }
    }

    @Test
    public void onTheUpCellsHalfItShoots() {
        for (Cell cell : Cell.values()) {
            assertEquals(FireControl.Mode.SHOOT, Robot.decideAimMode(cell, cell, true));
            assertEquals(FireControl.Mode.SHOOT, Robot.decideAimMode(cell, cell, false));
        }
    }

    @Test
    public void fromTheOtherHalfItPasses() {
        assertEquals(FireControl.Mode.PASS, Robot.decideAimMode(Cell.FAR, Cell.AUDIENCE, true));
        assertEquals(FireControl.Mode.PASS, Robot.decideAimMode(Cell.AUDIENCE, Cell.FAR, true));
    }

    @Test
    public void withPassingOffTheOtherHalfDoesNothing() {
        assertEquals(FireControl.Mode.NONE, Robot.decideAimMode(Cell.FAR, Cell.AUDIENCE, false));
        assertEquals(FireControl.Mode.NONE, Robot.decideAimMode(Cell.AUDIENCE, Cell.FAR, false));
    }
}
