package org.firstinspires.ftc.teamcode.agateflow;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.PathSegment;

import org.firstinspires.ftc.teamcode.field.Alliance;

import java.util.ArrayList;
import java.util.List;

/** Checks the tests run on a Plan: room by the true rule (TrueRule) along the planned and predicted paths. */
final class PlanChecks {
    private PlanChecks() {}

    /** The route points the true rule relaxes near: the start, then every leg's waypoints and its end. */
    static double[] points(Pose start, Plan plan, Alliance alliance) {
        List<Double> xy = new ArrayList<>();
        xy.add(start.x());
        xy.add(start.y());
        for (Leg leg : plan.legs) {
            for (Waypoint w : leg.waypoints) {
                Waypoint.Resolved r = w.resolve(alliance);
                if (r == null) continue;
                xy.add(r.x);
                xy.add(r.y);
            }
            xy.add(leg.end.x());
            xy.add(leg.end.y());
        }
        double[] out = new double[xy.size()];
        for (int i = 0; i < out.length; i++) out[i] = xy.get(i);
        return out;
    }

    /** Least room along the Pedro paths (every half inch), facing the path's heading there. */
    static double plannedRoom(Plan plan, List<Zone> zones, double radius, double[] points, double startHeading) {
        double least = Double.POSITIVE_INFINITY;
        for (Leg leg : plan.legs) {
            if (leg.path == null) continue;
            for (PathSegment segment : leg.path.getSegments()) {
                int n = Math.max(4, (int) Math.ceil(segment.curve.length() / 0.5));
                for (int k = 0; k <= n; k++) {
                    Vector2D p = segment.curve.get((double) k / n);
                    least = Math.min(least, TrueRule.room(zones, radius, points, startHeading, p.x(), p.y(), segment.heading((double) k / n)));
                }
            }
        }
        return least;
    }

    /** Least room along where AgateFlow predicts the robot really goes. */
    static double predictedRoom(Plan plan, List<Zone> zones, double radius, double[] points, double startHeading) {
        double least = Double.POSITIVE_INFINITY;
        for (Leg leg : plan.legs) {
            for (int q = 0; q + 1 < leg.predictedPath.length; q += 2) {
                least = Math.min(least, TrueRule.room(zones, radius, points, startHeading,
                        leg.predictedPath[q], leg.predictedPath[q + 1], leg.predictedHeading[q / 2]));
            }
        }
        return least;
    }
}
