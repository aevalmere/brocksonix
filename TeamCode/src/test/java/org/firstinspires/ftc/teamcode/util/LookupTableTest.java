package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

/** LookupTable: a straight line between rows, the end value past either end, and the table's range. */
public class LookupTableTest {
    private static final double EPS = 1e-9;
    private static final double[][] TABLE = {
            {20, 2000},
            {50, 2600},
            {110, 3800},
    };

    @Test
    public void rowsGiveTheirOwnValue() {
        assertEquals(2000, LookupTable.at(TABLE, 20), EPS);
        assertEquals(2600, LookupTable.at(TABLE, 50), EPS);
        assertEquals(3800, LookupTable.at(TABLE, 110), EPS);
    }

    @Test
    public void betweenRowsIsAStraightLine() {
        // A third of the way from 20 to 50, and halfway from 50 to 110.
        assertEquals(2200, LookupTable.at(TABLE, 30), EPS);
        assertEquals(3200, LookupTable.at(TABLE, 80), EPS);
    }

    @Test
    public void pastEitherEndGivesTheEndValue() {
        assertEquals(2000, LookupTable.at(TABLE, 0), EPS);
        assertEquals(3800, LookupTable.at(TABLE, 500), EPS);
    }

    @Test
    public void minAndMaxAreTheFirstAndLastDistances() {
        assertEquals(20, LookupTable.minX(TABLE), EPS);
        assertEquals(110, LookupTable.maxX(TABLE), EPS);
    }

    @Test
    public void aOneRowTableGivesThatRowEverywhere() {
        double[][] one = {{40, 7}};
        assertEquals(7, LookupTable.at(one, 0), EPS);
        assertEquals(7, LookupTable.at(one, 40), EPS);
        assertEquals(7, LookupTable.at(one, 90), EPS);
    }
}
