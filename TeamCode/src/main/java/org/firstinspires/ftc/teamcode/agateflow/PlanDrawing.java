package org.firstinspires.ftc.teamcode.agateflow;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.PathSegment;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.IntakePoses;

import java.util.List;
import java.util.Locale;

/**
 * Draws a Plan: the zones, the path Pedro follows, where AgateFlow predicts the
 * robot really goes, pickUp targets (picked up or skipped) and the robot's
 * outline at each stop and where it is now. On the Panels field page
 * (toPanels), or as an SVG picture (toSvg, the tests save these).
 *
 * <pre>
 * // in the loop, after robot.update(). It sends at most every 100 ms by itself.
 * PlanDrawing.toPanels(drive.plan(), agateFlow.map(), follower.pose());
 * </pre>
 *
 * Colors (they show on Panels' dark field picture and on a light one): zones
 * and the field's edge, where the walls start, magenta; the planned path blue;
 * the predicted path green; the robot's outline orange with its intake end
 * red, thin at each stop (reaching as far as the flower intake where
 * Route.flowerIntakeDown has it down) and thick where the robot is now;
 * picked-up targets green, skipped ones red.
 *
 * Every Panels packet carries the whole drawing over Wi-Fi: about 170 lines
 * and 25 KB for a route with 4 stops. So the paths are drawn in 2 in. pieces,
 * and an OpMode that only shows a plan should draw it once (see
 * AgateFlowBenchmark), not every loop.
 */
public final class PlanDrawing {
    private PlanDrawing() {}

    /**
     * Panels draws its field picture 144 of its inches across, and its PEDRO_PATHING
     * preset puts 0 and 144 on the picture's edges. Our field is Field.SIZE across
     * (tile edge to tile edge, which is the picture's edge), so every point and length
     * sent to Panels is scaled by this: our 0 and Field.SIZE land on the picture's
     * edges and our center on its center. Only for drawing; planning never uses 144.
     */
    static final double TO_PANELS = 144 / Field.SIZE;
    /** Paths are drawn as straight pieces about this long, inches. */
    static final double PIECE = 2;

    static final String ZONE = "magenta", PLANNED = "dodgerblue", PREDICTED = "limegreen", ROBOT = "orange", INTAKE_END = "red",
            PICKED_UP = "limegreen", SKIPPED = "red";

    /** Where a drawing goes: straight lines and circles in field inches. */
    interface Canvas {
        void line(double x1, double y1, double x2, double y2, String color, double width);

        void circle(double x, double y, double radius, String color, double width, boolean filled);
    }

    /** Draws the plan (may be null) and the zones, and the robot's outline at {@code robot} (may be null). */
    static void draw(Plan plan, FieldMap.Snapshot zones, Pose robot, Canvas canvas) {
        // The walls are blocks just outside the field (FieldMap), so draw where they start.
        double s = Field.SIZE;
        canvas.line(0, 0, s, 0, ZONE, 0.6);
        canvas.line(s, 0, s, s, ZONE, 0.6);
        canvas.line(s, s, 0, s, ZONE, 0.6);
        canvas.line(0, s, 0, 0, ZONE, 0.6);
        for (Zone zone : zones.zones) {
            if (zone.wall) continue;
            for (int i = 0; i < zone.xs.length; i++) {
                int j = (i + 1) % zone.xs.length;
                canvas.line(zone.xs[i], zone.ys[i], zone.xs[j], zone.ys[j], ZONE, 0.6);
            }
        }
        if (plan != null && plan.ok()) {
            for (Leg leg : plan.legs) {
                if (leg.path != null) {
                    for (PathSegment segment : leg.path.getSegments()) {
                        int n = Math.max(2, (int) Math.ceil(segment.curve.length() / PIECE));
                        Vector2D last = segment.curve.get(0);
                        for (int k = 1; k <= n; k++) {
                            Vector2D p = segment.curve.get((double) k / n);
                            canvas.line(last.x(), last.y(), p.x(), p.y(), PLANNED, 0.8);
                            last = p;
                        }
                    }
                }
                predicted(leg.predictedPath, canvas);
                List<Waypoint> waypoints = leg.waypoints;
                boolean flowerDown = waypoints.get(waypoints.size() - 1).flowerDown;
                outline(leg.end, flowerDown ? IntakePoses.FLOWER_INTAKE_REACH : AgateFlow.ROBOT_FRONT, 0.4, canvas);
            }
            for (Pose p : plan.pickedUp()) canvas.circle(p.x(), p.y(), AgateFlow.BALL_RADIUS, PICKED_UP, 0.4, true);
            for (Pose p : plan.skipped()) canvas.circle(p.x(), p.y(), AgateFlow.BALL_RADIUS, SKIPPED, 0.4, true);
        }
        if (robot != null) outline(robot, AgateFlow.ROBOT_FRONT, 0.8, canvas);
    }

    /** The predicted path (x, y pairs, about every half inch), as pieces about PIECE long. */
    private static void predicted(double[] path, Canvas canvas) {
        if (path.length < 4) return;
        double lastX = path[0], lastY = path[1];
        for (int q = 2; q + 1 < path.length; q += 2) {
            boolean end = q + 2 >= path.length;
            if (!end && Geometry.length(path[q] - lastX, path[q + 1] - lastY) < PIECE) continue;
            canvas.line(lastX, lastY, path[q], path[q + 1], PREDICTED, 0.5);
            lastX = path[q];
            lastY = path[q + 1];
        }
    }

    /** The robot's outline at this pose, reaching {@code front} ahead of its center, its intake end red. */
    private static void outline(Pose robot, double front, double width, Canvas canvas) {
        double c = Math.cos(robot.heading()), s = Math.sin(robot.heading());
        double b = AgateFlow.ROBOT_BACK, w = AgateFlow.ROBOT_HALF_WIDTH;
        double[][] corners = {{front, w}, {-b, w}, {-b, -w}, {front, -w}};
        double[] x = new double[4], y = new double[4];
        for (int i = 0; i < 4; i++) {
            x[i] = robot.x() + corners[i][0] * c - corners[i][1] * s;
            y[i] = robot.y() + corners[i][0] * s + corners[i][1] * c;
        }
        for (int i = 0; i < 4; i++) {
            int j = (i + 1) % 4;
            canvas.line(x[i], y[i], x[j], y[j], i == 3 ? INTAKE_END : ROBOT, width);
        }
    }

    /**
     * Draws on the Panels field page (PEDRO_PATHING preset) and sends it. Panels
     * sends at most every 100 ms, so between sends this does nothing and returns
     * false: calling it every loop costs almost nothing. It sends by itself (it
     * calls the field's update()), so nothing else should draw on the field page
     * in the same OpMode.
     *
     * @return true if the drawing was sent
     */
    public static boolean toPanels(Plan plan, FieldMap map, Pose robot) {
        FieldManager field = PanelsField.INSTANCE.getField();
        if (!field.getShouldUpdateCanvas()) return false;
        toPanels(field, plan, map.snapshot(), robot);
        field.update();
        return true;
    }

    /** Adds the drawing to the field's canvas, in Panels inches, without sending it. */
    static void toPanels(final FieldManager field, Plan plan, FieldMap.Snapshot zones, Pose robot) {
        final String none = PanelsField.INSTANCE.getTRANSPARENT();
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        draw(plan, zones, robot, new Canvas() {
            @Override
            public void line(double x1, double y1, double x2, double y2, String color, double width) {
                field.setStyle(none, color, panels(width));
                field.moveCursor(panels(x1), panels(y1));
                field.line(panels(x2), panels(y2));
            }

            @Override
            public void circle(double x, double y, double radius, String color, double width, boolean filled) {
                field.setStyle(filled ? color : none, color, panels(width));
                field.moveCursor(panels(x), panels(y));
                field.circle(panels(radius));
            }
        });
    }

    /** Field inches to Panels inches, to a hundredth: short numbers keep the packets small. */
    static double panels(double inches) {
        return Math.round(inches * TO_PANELS * 100) / 100.0;
    }

    /** The drawing as an SVG picture, {@code pixelsPerInch} to the inch, +y up, on Panels' dark field color. */
    static String toSvg(Plan plan, FieldMap.Snapshot zones, Pose robot, double pixelsPerInch) {
        final double k = pixelsPerInch, size = Field.SIZE * k;
        final StringBuilder svg = new StringBuilder();
        svg.append(String.format(Locale.ROOT, "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%.0f\" height=\"%.0f\" viewBox=\"0 0 %.1f %.1f\">%n",
                size, size, size, size));
        svg.append(String.format(Locale.ROOT, "<rect x=\"0\" y=\"0\" width=\"%.1f\" height=\"%.1f\" fill=\"#2a2a2a\"/>%n", size, size));
        draw(plan, zones, robot, new Canvas() {
            @Override
            public void line(double x1, double y1, double x2, double y2, String color, double width) {
                svg.append(String.format(Locale.ROOT, "<line x1=\"%.2f\" y1=\"%.2f\" x2=\"%.2f\" y2=\"%.2f\" stroke=\"%s\" stroke-width=\"%.2f\"/>%n",
                        x1 * k, size - y1 * k, x2 * k, size - y2 * k, color, width * k));
            }

            @Override
            public void circle(double x, double y, double radius, String color, double width, boolean filled) {
                svg.append(String.format(Locale.ROOT, "<circle cx=\"%.2f\" cy=\"%.2f\" r=\"%.2f\" fill=\"%s\" stroke=\"%s\" stroke-width=\"%.2f\"/>%n",
                        x * k, size - y * k, radius * k, filled ? color : "none", color, width * k));
            }
        });
        svg.append("</svg>\n");
        return svg.toString();
    }
}
