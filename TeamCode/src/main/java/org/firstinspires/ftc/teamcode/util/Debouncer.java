package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

/** Ignores flickers: the value only changes after it reads the same new value for a number of milliseconds. */
public class Debouncer {
    private boolean stable;
    private boolean changing = false;
    private final ElapsedTime changingFor = new ElapsedTime();

    public Debouncer(boolean initial) {
        stable = initial;
    }

    /** holdMs is read on every call, so it can be changed live. */
    public boolean update(boolean raw, double holdMs) {
        if (raw == stable) {
            changing = false;
        } else {
            // The timer starts at the first reading that differs, and any reading that matches again cancels it.
            if (!changing) {
                changing = true;
                changingFor.reset();
            }
            if (changingFor.milliseconds() >= holdMs) {
                stable = raw;
                changing = false;
            }
        }
        return stable;
    }

    public boolean get() {
        return stable;
    }
}
