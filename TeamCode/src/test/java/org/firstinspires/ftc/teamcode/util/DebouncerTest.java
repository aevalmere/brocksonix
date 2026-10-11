package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 * Debouncer: the value only changes after a new reading holds for holdMs. Debouncer
 * runs on the real clock, so these use a hold of 0 (changes at once) or a long one
 * (never changes during the test).
 */
public class DebouncerTest {
    private static final double NEVER_MS = 1e9;

    @Test
    public void startsAtTheInitialValue() {
        assertFalse(new Debouncer(false).get());
        assertTrue(new Debouncer(true).get());
    }

    @Test
    public void aHoldOfZeroChangesAtOnce() {
        Debouncer debouncer = new Debouncer(false);
        assertTrue(debouncer.update(true, 0));
        assertFalse(debouncer.update(false, 0));
    }

    @Test
    public void aFlickerShorterThanTheHoldIsIgnored() {
        Debouncer debouncer = new Debouncer(false);
        assertFalse(debouncer.update(true, NEVER_MS));
        assertFalse(debouncer.update(false, NEVER_MS));
        assertFalse(debouncer.get());
    }

    @Test
    public void holdMsIsReadOnEveryCall() {
        Debouncer debouncer = new Debouncer(false);
        assertFalse(debouncer.update(true, NEVER_MS));
        // The same wait, with the hold lowered live (a Panels edit): it is over now.
        assertTrue(debouncer.update(true, 0));
    }
}
