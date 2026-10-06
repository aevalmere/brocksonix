package org.firstinspires.ftc.teamcode.util;

/** Ignores flickers: the value only changes after it reads the same new value several loops in a row. */
public class Debouncer {
    private boolean stable;
    private int differentCount = 0;

    public Debouncer(boolean initial) {
        stable = initial;
    }

    public boolean update(boolean raw, int loopsRequired) {
        if (raw == stable) {
            differentCount = 0;
        } else if (++differentCount >= loopsRequired) {
            stable = raw;
            differentCount = 0;
        }
        return stable;
    }

    public boolean get() {
        return stable;
    }
}
