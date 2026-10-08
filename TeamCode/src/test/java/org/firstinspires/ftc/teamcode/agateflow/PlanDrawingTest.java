package org.firstinspires.ftc.teamcode.agateflow;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.bylazar.field.Circle;
import com.bylazar.field.Drawable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.Line;
import com.bylazar.field.PanelsField;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.Flower;
import org.firstinspires.ftc.teamcode.field.IntakePoses;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;

/**
 * PlanDrawing: the Panels scale, what Panels gets, how big its packets are, and
 * pictures of a few plans saved as SVG in TeamCode/build/agateflow-svg (open them
 * in a browser to see what AgateFlow does).
 */
public class PlanDrawingTest {
    private static final Pose SHOOT = new Pose(36, 48, Math.PI);
    private double timeLimit;

    @Before
    public void noTimeLimit() {
        timeLimit = AgateFlow.PLAN_TIME_LIMIT_MS;
        AgateFlow.PLAN_TIME_LIMIT_MS = 1e9;
    }

    @After
    public void putBack() {
        AgateFlow.PLAN_TIME_LIMIT_MS = timeLimit;
    }

    @Test
    public void panelsScalePutsOurFieldOnItsPicture() {
        // Panels' picture is 144 of its inches across: our edges and center must land on its edges and center.
        assertEquals(144, Field.SIZE * PlanDrawing.TO_PANELS, 1e-9);
        assertEquals(72, Field.SIZE / 2 * PlanDrawing.TO_PANELS, 1e-9);
    }

    /** One line or circle as drawn, in field inches. */
    private static final class Shape {
        final double[] numbers;
        final String color;

        Shape(String color, double... numbers) {
            this.color = color;
            this.numbers = numbers;
        }
    }

    private static List<Shape> drawn(Plan plan, AgateFlow agateFlow, Pose robot) {
        final List<Shape> shapes = new ArrayList<>();
        PlanDrawing.draw(plan, agateFlow.map().snapshot(), robot, new PlanDrawing.Canvas() {
            @Override
            public void line(double x1, double y1, double x2, double y2, String color, double width) {
                shapes.add(new Shape(color, x1, y1, x2, y2, width));
            }

            @Override
            public void circle(double x, double y, double radius, String color, double width, boolean filled) {
                shapes.add(new Shape(color, x, y, radius, width));
            }
        });
        return shapes;
    }

    @Test
    public void panelsGetsTheSameDrawingInItsInches() {
        TestRobot robot = new TestRobot();
        AgateFlow agateFlow = robot.agateFlow();
        Pose start = new Pose(20, 20, 0);
        Plan plan = robot.plan(agateFlow, start, new Route()
                .pickUp(new Pose(50, 30), new Pose(80, 40), new Pose(60, 110)).most(2).to(SHOOT), Alliance.RED);
        assertTrue(plan.toString(), plan.ok());

        FieldManager field = new FieldManager();
        PlanDrawing.toPanels(field, plan, agateFlow.map().snapshot(), start);
        List<Drawable> items = field.getCanvas().getItems();
        List<Shape> shapes = drawn(plan, agateFlow, start);
        assertEquals(shapes.size(), items.size());
        assertEquals(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING(), field.getCanvas().getPreset());
        for (int i = 0; i < items.size(); i++) {
            Shape shape = shapes.get(i);
            double[] got;
            String color;
            if (items.get(i) instanceof Line) {
                Line line = (Line) items.get(i);
                got = new double[]{line.getX1(), line.getY1(), line.getX2(), line.getY2(), line.getStyle().getOutlineWidth()};
                color = line.getStyle().getOutlineFill();
            } else {
                Circle circle = (Circle) items.get(i);
                got = new double[]{circle.getX(), circle.getY(), circle.getR(), circle.getStyle().getOutlineWidth()};
                color = circle.getStyle().getOutlineFill();
            }
            assertEquals(shape.color, color);
            assertEquals(shape.numbers.length, got.length);
            for (int k = 0; k < got.length; k++) {
                // Every number scaled by TO_PANELS, rounded to a hundredth, and on the picture (0 to 144).
                assertEquals(shape.numbers[k] * PlanDrawing.TO_PANELS, got[k], 0.005 + 1e-9);
                assertTrue(got[k] > -0.01 && got[k] < 144.01);
            }
        }
        // Two picked up (green), one skipped (red).
        int picked = 0, skipped = 0;
        for (Drawable item : items) {
            if (!(item instanceof Circle)) continue;
            if (((Circle) item).getStyle().getFill().equals(PlanDrawing.PICKED_UP)) picked++;
            if (((Circle) item).getStyle().getFill().equals(PlanDrawing.SKIPPED)) skipped++;
        }
        assertEquals(plan.pickedUp().size(), picked);
        assertEquals(plan.skipped().size(), skipped);
        assertEquals(2, picked);
        assertEquals(1, skipped);
    }

    @Test
    public void flowerIntakeDownStopIsDrawnWithItsReach() {
        TestRobot robot = new TestRobot();
        AgateFlow agateFlow = robot.agateFlow();
        Plan plan = robot.plan(agateFlow, SHOOT, new Route().to(IntakePoses.flowerPose(Flower.ALLIANCE_WALL))
                .approach(IntakePoses.flowerTravelDegrees(Flower.ALLIANCE_WALL), IntakePoses.FLOWER_APPROACH).flowerIntakeDown()
                .to(SHOOT), Alliance.RED);
        assertTrue(plan.toString(), plan.ok());
        assertEquals(2, plan.legs.size());
        // The intake end (red) of the outline at each stop: FLOWER_INTAKE_REACH ahead at the FLOWER, ROBOT_FRONT back at SHOOT.
        assertEquals(IntakePoses.FLOWER_INTAKE_REACH, intakeEndAhead(plan, agateFlow, 0), 1e-6);
        assertEquals(AgateFlow.ROBOT_FRONT, intakeEndAhead(plan, agateFlow, 1), 1e-6);
    }

    /** How far ahead of leg i's stop its outline's intake end is drawn, inches. */
    private static double intakeEndAhead(Plan plan, AgateFlow agateFlow, int i) {
        Pose end = plan.legs.get(i).end;
        double c = Math.cos(end.heading()), s = Math.sin(end.heading());
        double best = Double.NaN;
        for (Shape shape : drawn(plan, agateFlow, null)) {
            if (!shape.color.equals(PlanDrawing.INTAKE_END) || shape.numbers.length != 5) continue;
            double mx = (shape.numbers[0] + shape.numbers[2]) / 2 - end.x(), my = (shape.numbers[1] + shape.numbers[3]) / 2 - end.y();
            // The intake end's middle sits straight ahead of the stop.
            if (Math.abs(-mx * s + my * c) < 1e-6 && Math.abs(Math.hypot(mx, my) - (mx * c + my * s)) < 1e-6) best = mx * c + my * s;
        }
        return best;
    }

    /**
     * Each Panels packet carries the whole drawing over Wi-Fi. Roughly what its JSON
     * weighs, from the numbers and colors in it (Panels adds a little more).
     */
    private static int packetBytes(FieldManager field) {
        int bytes = 0;
        for (Drawable item : field.getCanvas().getItems()) {
            if (item instanceof Line) {
                Line l = (Line) item;
                bytes += 100 + ("" + l.getX1() + l.getY1() + l.getX2() + l.getY2() + l.getStyle().getOutlineWidth()
                        + l.getStyle().getOutlineFill() + l.getStyle().getFill()).length();
            } else {
                Circle c = (Circle) item;
                bytes += 100 + ("" + c.getX() + c.getY() + c.getR() + c.getStyle().getOutlineWidth()
                        + c.getStyle().getOutlineFill() + c.getStyle().getFill()).length();
            }
        }
        return bytes;
    }

    @Test
    public void panelsPacketsStaySmall() {
        TestRobot robot = new TestRobot();
        AgateFlow agateFlow = robot.agateFlow();
        // A long one: the GARDEN, a FLOWER, then shooting.
        Pose start = new Pose(36, 30, Math.PI);
        Plan plan = robot.plan(agateFlow, start, new Route().to(IntakePoses.gardenStartPose())
                .to(IntakePoses.gardenEndPose()).allowContact().maxSpeed(20)
                .to(IntakePoses.flowerPose(Flower.ALLIANCE_WALL)).approach(IntakePoses.flowerTravelDegrees(Flower.ALLIANCE_WALL), IntakePoses.FLOWER_APPROACH)
                .to(SHOOT), Alliance.RED);
        assertTrue(plan.toString(), plan.ok());
        FieldManager field = new FieldManager();
        PlanDrawing.toPanels(field, plan, agateFlow.map().snapshot(), start);
        int items = field.getCanvas().getItems().size(), bytes = packetBytes(field);
        System.out.printf("Panels drawing of a %.0f in. route: %d items, about %d KB a packet%n", plan.length(), items, bytes / 1000);
        assertTrue(items + " items", items < 400);
        assertTrue(bytes + " bytes", bytes < 50000);
    }

    private static void save(String name, Pose start, Route route) throws IOException {
        TestRobot robot = new TestRobot();
        AgateFlow agateFlow = robot.agateFlow();
        Plan plan = robot.plan(agateFlow, start, route, Alliance.RED);
        assertTrue(plan.toString(), plan.ok());
        String svg = PlanDrawing.toSvg(plan, agateFlow.map().snapshot(), start, 5);
        assertTrue(svg.startsWith("<svg") && svg.contains("stroke=\"" + PlanDrawing.PLANNED + "\"")
                && svg.contains("stroke=\"" + PlanDrawing.PREDICTED + "\""));
        File dir = new File("build/agateflow-svg");
        assertTrue(dir.isDirectory() || dir.mkdirs());
        Files.write(new File(dir, name + ".svg").toPath(), svg.getBytes(StandardCharsets.UTF_8));
    }

    @Test
    public void savesPictures() throws IOException {
        save("auto-start-to-shoot", new Pose(8, 24, Math.PI), new Route().to(SHOOT));
        save("flower-then-shoot", new Pose(30, 30, 0), new Route().to(IntakePoses.flowerPose(Flower.ALLIANCE_WALL))
                .approach(IntakePoses.flowerTravelDegrees(Flower.ALLIANCE_WALL), IntakePoses.FLOWER_APPROACH).allowContact().to(SHOOT));
        save("flower-intake-down-then-shoot", SHOOT, new Route().to(IntakePoses.flowerPose(Flower.ALLIANCE_WALL))
                .approach(IntakePoses.flowerTravelDegrees(Flower.ALLIANCE_WALL), IntakePoses.FLOWER_APPROACH).flowerIntakeDown()
                .to(SHOOT));
        save("pickup-three-then-shoot", new Pose(20, 20, 0),
                new Route().pickUp(new Pose(50, 30, 0), new Pose(80, 40, 0), new Pose(60, 110, 0)).to(SHOOT));
        save("ride-the-red-wall", new Pose(AgateFlow.ROBOT_HALF_WIDTH + 0.5, 62, Math.PI / 2),
                new Route().to(AgateFlow.ROBOT_HALF_WIDTH + 0.5, 128, 90));
    }
}
