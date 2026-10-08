package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.agateflow.AgateFlow;
import org.firstinspires.ftc.teamcode.agateflow.DriveModel;
import org.firstinspires.ftc.teamcode.agateflow.FieldMap;
import org.firstinspires.ftc.teamcode.agateflow.Heading;
import org.firstinspires.ftc.teamcode.agateflow.Plan;
import org.firstinspires.ftc.teamcode.agateflow.PlanDrawing;
import org.firstinspires.ftc.teamcode.agateflow.Route;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.FieldZones;
import org.firstinspires.ftc.teamcode.field.Flower;
import org.firstinspires.ftc.teamcode.field.IntakePoses;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Times AgateFlow's planning on this Control Hub. It only plans, it never
 * drives: the robot can sit on a cart. Press start, wait for "done", and
 * read the times. Planning runs on the AgateFlow thread during a match, so
 * the loop never waits for it, but a slow plan means a late start or a late
 * replan. Compare the times with AgateFlow.PLAN_TIME_LIMIT_MS: plans that hit
 * it stop trying more routes and may be a little slower to drive.
 *
 * When it is done, each plan can be looked at on Panels' field page (PEDRO
 * PATHING preset): D-pad left and right pick the route, D-pad up switches
 * red/blue. See PlanDrawing for the colors.
 *
 * Before Pedro is tuned (Constants.create returns null) it plans with
 * DriveModel's fallback numbers, which takes about as long. But with no
 * follower AgateFlow can't add speed limits, so routes that need a slow part
 * (the FLOWER and GARDEN ones) find no way.
 */
@TeleOp(name = "AgateFlow Benchmark", group = "Test")
public class AgateFlowBenchmark extends LinearOpMode {
    /** Each route is planned this many times for each alliance. */
    private static final int ROUNDS = 3;

    @Override
    public void runOpMode() {
        Telemetry out = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        Follower follower = Constants.create(hardwareMap);
        out.addLine(follower == null ? "Pedro isn't set up yet: planning with DriveModel's fallback numbers and no speed limits, so the FLOWER and GARDEN routes find no way." : "Planning with this robot's Foresight.");
        out.addLine("Press start. The robot doesn't move.");
        out.update();
        waitForStart();

        FieldMap map = new FieldMap();
        map.setZones("field", FieldZones.zones());
        AgateFlow agateFlow = new AgateFlow(map, follower);
        DriveModel model = DriveModel.capture(follower);
        List<Pose> starts = new ArrayList<>();
        List<Route> routes = new ArrayList<>();
        routes(starts, routes);

        List<Double> millis = new ArrayList<>();
        int failed = 0, overLimit = 0;
        // The last plan of each route, for each alliance (red first), to draw when done.
        Plan[][] plans = new Plan[2][routes.size()];
        for (int round = 0; round < ROUNDS && opModeIsActive(); round++) {
            for (Alliance alliance : Alliance.values()) {
                for (int i = 0; i < routes.size() && opModeIsActive(); i++) {
                    Pose start = alliance.fromRed(starts.get(i));
                    Plan plan = agateFlow.plan(start, new Velocity(0, 0, 0), routes.get(i), alliance, 12.5, model);
                    plans[alliance == Alliance.RED ? 0 : 1][i] = plan;
                    millis.add(plan.planMillis);
                    if (!plan.ok()) failed++;
                    if (plan.planMillis >= AgateFlow.PLAN_TIME_LIMIT_MS) overLimit++;
                    out.addData("Planning", "%d of %d", millis.size(), ROUNDS * 2 * routes.size());
                    out.addData("Last", "%s", plan);
                    out.update();
                }
            }
        }

        Collections.sort(millis);
        int shown = 0;
        Alliance alliance = Alliance.RED;
        boolean drawn = false;
        while (opModeIsActive()) {
            if (gamepad1.dpadRightWasPressed()) {
                shown = (shown + 1) % routes.size();
                drawn = false;
            }
            if (gamepad1.dpadLeftWasPressed()) {
                shown = (shown + routes.size() - 1) % routes.size();
                drawn = false;
            }
            if (gamepad1.dpadUpWasPressed()) {
                alliance = alliance.other();
                drawn = false;
            }
            Plan plan = plans[alliance == Alliance.RED ? 0 : 1][shown];
            // Drawn once per choice: every Panels packet carries the whole drawing over Wi-Fi.
            if (!drawn && plan != null) drawn = PlanDrawing.toPanels(plan, map, alliance.fromRed(starts.get(shown)));

            out.addLine("Done.");
            if (!millis.isEmpty()) {
                out.addData("Plans", "%d (%d found no way)", millis.size(), failed);
                out.addData("Median ms", "%.0f", millis.get(millis.size() / 2));
                out.addData("90th percentile ms", "%.0f", millis.get((int) (millis.size() * 0.9)));
                out.addData("Slowest ms", "%.0f", millis.get(millis.size() - 1));
                out.addData("Hit PLAN_TIME_LIMIT_MS", "%d of %d (limit %.0f ms)", overLimit, millis.size(), AgateFlow.PLAN_TIME_LIMIT_MS);
            }
            out.addLine("");
            out.addData("On Panels' field (D-pad left/right, up)", "route %d of %d, %s", shown + 1, routes.size(), alliance);
            if (plan != null) {
                out.addData("Plan", "%s", plan);
                for (String note : plan.notes) out.addLine(note);
            }
            out.update();
            sleep(50);
        }
    }

    /** Routes like an auto uses them, written for red, with where each starts. AgateFlowBenchmarkTest checks they all plan. */
    public static void routes(List<Pose> starts, List<Route> routes) {
        Pose shoot = new Pose(36, 48, Math.PI);
        add(starts, routes, new Pose(8, 24, Math.PI), new Route().to(shoot));
        // At a FLOWER the flower intake is down, so AgateFlow plans the way in with it reaching further ahead.
        add(starts, routes, new Pose(30, 30, 0), new Route().to(IntakePoses.flowerPose(Flower.ALLIANCE_WALL))
                .approach(IntakePoses.flowerTravelDegrees(Flower.ALLIANCE_WALL), IntakePoses.FLOWER_APPROACH).flowerIntakeDown());
        add(starts, routes, new Pose(30, 30, 0), new Route().to(IntakePoses.flowerPose(Flower.LEFT_WALL))
                .approach(IntakePoses.flowerTravelDegrees(Flower.LEFT_WALL), IntakePoses.FLOWER_APPROACH).flowerIntakeDown());
        add(starts, routes, IntakePoses.flowerPose(Flower.ALLIANCE_WALL), new Route().to(shoot));
        add(starts, routes, IntakePoses.flowerPose(Flower.LEFT_WALL), new Route().to(shoot));
        add(starts, routes, new Pose(40, 40, 0), new Route().to(IntakePoses.gardenStartPose())
                .to(IntakePoses.gardenEndPose()).allowContact().maxSpeed(20));
        add(starts, routes, new Pose(20, 20, 0), new Route().to(120, 120).heading(Heading.tangent()));
        add(starts, routes, new Pose(30, 70, 0), new Route().to(112, 70));
        add(starts, routes, new Pose(60, 60, 0), new Route().to(32, 70));
        add(starts, routes, new Pose(10, 10, 0), new Route().through(40, 100, 90).heading(Heading.tangent())
                .to(100, 120, 0).heading(Heading.finishBy(0.5)));
        add(starts, routes, new Pose(36, 30, Math.PI), new Route().to(IntakePoses.gardenStartPose())
                .to(IntakePoses.gardenEndPose()).allowContact().maxSpeed(20)
                .to(IntakePoses.flowerPose(Flower.ALLIANCE_WALL)).approach(IntakePoses.flowerTravelDegrees(Flower.ALLIANCE_WALL), IntakePoses.FLOWER_APPROACH)
                .flowerIntakeDown());
        add(starts, routes, new Pose(20, 120, 0), new Route().to(70.75, 70.75, 90));
        // Floor pickups: 2 of 3, the quickest, then go shoot (pickUp's ordering takes the most planning time).
        add(starts, routes, new Pose(20, 20, 0), new Route().pickUp(new Pose(50, 30), new Pose(80, 40), new Pose(60, 110)).most(2)
                .to(shoot));
    }

    private static void add(List<Pose> starts, List<Route> routes, Pose start, Route route) {
        starts.add(start);
        routes.add(route);
    }
}
