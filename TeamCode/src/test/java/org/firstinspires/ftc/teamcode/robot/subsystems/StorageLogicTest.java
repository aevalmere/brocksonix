package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** What Storage reports for each pattern of its beams at slots 1, 3 and 4 (full). */
public class StorageLogicTest {

    @Test
    public void ballsStackedFromTheDoor() {
        assertEquals("0", Storage.countLabel(false, false, false));
        // 1 or 2 balls: only slot 1 is seen, slot 2 has no beam.
        assertEquals("1-2", Storage.countLabel(true, false, false));
        // 3 balls: slots 1 and 3.
        assertEquals("2-3", Storage.countLabel(true, true, false));
        assertEquals("4", Storage.countLabel(true, true, true));
    }

    @Test
    public void ballsSpreadOut() {
        // A ball at slot 3 with slot 1 clear: maybe another at slot 2.
        assertEquals("1-2", Storage.countLabel(false, true, false));
        // Full wins whatever the others read.
        assertEquals("4", Storage.countLabel(false, false, true));
    }

    @Test
    public void emptyOnlyWhenNoBeamSeesABall() {
        assertTrue(Storage.isEmpty(false, false, false));
        assertFalse(Storage.isEmpty(true, false, false));
        assertFalse(Storage.isEmpty(false, true, false));
        assertFalse(Storage.isEmpty(false, false, true));
    }
}
