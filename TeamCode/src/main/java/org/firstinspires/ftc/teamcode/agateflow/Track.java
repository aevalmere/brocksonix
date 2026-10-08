package org.firstinspires.ftc.teamcode.agateflow;

/**
 * A leg's path sampled about every inch: where it goes, which way the robot
 * faces, and any speed limit. MotionModel drives along it to estimate speeds
 * and times. Made by AgateFlow.
 */
final class Track {
    final int count;
    /** Distance along the leg, inches. */
    final double[] s;
    final double[] x, y;
    /** Direction of travel, radians. */
    final double[] travel;
    /** Curvature, 1/inches, positive turning left. */
    final double[] bend;
    /** Robot heading, radians, unwrapped (it never jumps by 2π). */
    final double[] heading;
    /** Heading change per inch of travel, radians/inch. */
    final double[] turnRate;
    /** Speed limit, inches/second. +infinity if none. */
    final double[] cap;
    /** Which stretch (between two waypoints) each sample is on, and which piece of that stretch. */
    final int[] stretch, piece;
    /**
     * Distance along the planned path. The same as s for the planned track; for
     * a predicted (driven) track, the planned distance at the nearest planned point.
     */
    final double[] plannedS;

    Track(int count) {
        this.count = count;
        s = new double[count];
        x = new double[count];
        y = new double[count];
        travel = new double[count];
        bend = new double[count];
        heading = new double[count];
        turnRate = new double[count];
        cap = new double[count];
        stretch = new int[count];
        piece = new int[count];
        plannedS = new double[count];
    }

    /** Index of the last sample whose planned distance is at or before d. */
    int indexAtPlanned(double d) {
        int lo = 0, hi = count - 1;
        if (d <= plannedS[0]) return 0;
        if (d >= plannedS[hi]) return hi;
        while (hi - lo > 1) {
            int mid = (lo + hi) >>> 1;
            if (plannedS[mid] <= d) lo = mid;
            else hi = mid;
        }
        return lo;
    }

    double length() {
        return s[count - 1];
    }

    /** Whether any sample has a speed limit (worked out once, after the track is filled in). */
    private int limits = -1;

    boolean hasLimits() {
        if (limits < 0) {
            limits = 0;
            for (int i = 0; i < count; i++) {
                if (cap[i] != Double.POSITIVE_INFINITY) {
                    limits = 1;
                    break;
                }
            }
        }
        return limits == 1;
    }

    /** Index of the last sample at or before distance d along the leg. */
    int indexAt(double d) {
        int lo = 0, hi = count - 1;
        if (d <= s[0]) return 0;
        if (d >= s[hi]) return hi;
        while (hi - lo > 1) {
            int mid = (lo + hi) >>> 1;
            if (s[mid] <= d) lo = mid;
            else hi = mid;
        }
        return lo;
    }
}
