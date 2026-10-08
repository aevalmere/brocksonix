package org.firstinspires.ftc.teamcode.agateflow;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.curves.Curve;

import java.util.Collections;
import java.util.List;

/**
 * One part of a Plan: from a stop (or the start) to the next stop, through any
 * through-waypoints in between. It is one Pedro Path, followed with one
 * follower.follow() call.
 */
public final class Leg {
    /** The Pedro path to follow. Null when the robot is already at the spot and only has to turn: hold {@link #end}. */
    public final Path path;
    /** Where the leg ends, field coordinates. */
    public final Pose end;
    /** Predicted time from the start of the leg until the follower is settled at the end, seconds. Not calibrated. */
    public final double seconds;
    /** Path length, inches. */
    public final double length;
    /** Fastest predicted speed, inches/second. */
    public final double topSpeed;
    /** Lowest predicted voltage at the motors (battery sag while speeding up), volts. */
    public final double lowestVolts;
    /** Highest predicted drive current, amps. */
    public final double peakAmps;
    /**
     * Least room between where the robot is predicted to really drive and the
     * clearance rule, inches. 0 or more is safe. A little below 0 only when no way
     * kept the whole SAFETY_MARGIN and AgateFlow took a near miss (see
     * AgateFlow.NEAR_MISS_FRACTION and Plan.notes).
     */
    public final double worstRoom;
    /** Segments AgateFlow gave a speed limit so the robot doesn't cut a corner into a zone. */
    public final int speedLimitedSegments;

    final Track track;
    final MotionModel.Profile profile;
    /** The Pedro segments in follow order, with where each starts along the leg. */
    final Curve[] segments;
    final double[] segmentStart;
    /** The route waypoints this leg drives to, in order, and how far along the leg each one is. */
    final List<Waypoint> waypoints;
    final double[] waypointAt;
    /** Where Foresight is expected to really drive, as x, y pairs. */
    final double[] predictedPath;
    /** The robot's predicted heading at each point of predictedPath, radians. */
    final double[] predictedHeading;
    /** The robot's predicted speed at each point of predictedPath, in/s. */
    final double[] predictedSpeed;

    Leg(Path path, Pose end, double seconds, double length, double topSpeed, double lowestVolts, double peakAmps,
        double worstRoom, int speedLimitedSegments, Track track, MotionModel.Profile profile,
        Curve[] segments, double[] segmentStart, List<Waypoint> waypoints, double[] waypointAt, double[] predictedPath,
        double[] predictedHeading, double[] predictedSpeed) {
        this.path = path;
        this.end = end;
        this.seconds = seconds;
        this.length = length;
        this.topSpeed = topSpeed;
        this.lowestVolts = lowestVolts;
        this.peakAmps = peakAmps;
        this.worstRoom = worstRoom;
        this.speedLimitedSegments = speedLimitedSegments;
        this.track = track;
        this.profile = profile;
        this.segments = segments;
        this.segmentStart = segmentStart;
        this.waypoints = Collections.unmodifiableList(waypoints);
        this.waypointAt = waypointAt;
        this.predictedPath = predictedPath;
        this.predictedHeading = predictedHeading;
        this.predictedSpeed = predictedSpeed;
    }

    /** The route waypoints this leg drives to, in order. The last one is the stop. */
    public List<Waypoint> waypoints() {
        return waypoints;
    }

    /**
     * How far along this leg the robot is, inches, while the follower follows
     * {@link #path}. Uses the follower's own closest point. Foresight moves on to
     * the next segment as soon as its brake point reaches the end of this one, up
     * to a brake distance (inches) before the robot gets there; then its closest
     * point sits at the new segment's start. So while that is so, this looks back
     * at the segments before for where the robot really is.
     */
    public double distanceDone(Follower follower) {
        if (path == null || segments.length == 0) return 0;
        if (!follower.following()) return length;
        int i = follower.pathIndex();
        if (i < 0) return 0;
        if (i >= segments.length) return length;
        double t = Math.max(0, Math.min(1, follower.parametricCompletion()));
        Vector2D at = follower.pose().toVector2D();
        while (i > 0 && t < 1e-6) {
            double before = segments[i - 1].closestParameter(at, 1);
            if (before >= 1 - 1e-6) break;
            i--;
            t = before;
        }
        return segmentStart[i] + segments[i].length() * segments[i].pathCompletion(t);
    }

    /**
     * Predicted seconds left on this leg, from where the follower is now, while it
     * follows {@link #path} or settles at its end. Not calibrated.
     */
    public double secondsLeft(Follower follower) {
        if (path == null) return follower.isBusy() ? seconds : 0;
        if (!follower.following()) return follower.isBusy() ? profile.seconds - profile.handoffSeconds : 0;
        return profile.secondsLeftFrom(track, distanceDone(follower));
    }

    /** Predicted seconds left on this leg from this far along it (inches), as planned. Not calibrated. */
    double secondsLeftFrom(double distance) {
        if (path == null) return seconds;
        return profile.secondsLeftFrom(track, distance);
    }

    /** Predicted seconds from Foresight's handoff (path done, holding the end) until the robot is at the stop. */
    double settleSeconds() {
        return profile == null ? 0 : profile.seconds - profile.handoffSeconds;
    }

    /** Predicted speed at this distance along the leg, inches/second. */
    public double predictedSpeedAt(double distance) {
        if (track == null) return 0;
        return profile.speed[track.indexAtPlanned(distance)];
    }

    /** Where Foresight is predicted to really drive, as x, y pairs (for drawing). Empty for a turn in place. */
    public double[] predictedPath() {
        return predictedPath.clone();
    }
}
