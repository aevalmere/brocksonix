package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.hardware.HardwareNames;

/**
 * Finds servo positions (door open/closed, flower intake up/down). Move one
 * servo at a time and copy the number into Door or FlowerIntake.
 *
 * D-pad left/right: pick a servo. D-pad up/down: ±0.01. Bumpers: ±0.05.
 * Servos don't move until you press something, so nothing jumps at start.
 */
@TeleOp(name = "Servo Position Finder", group = "Test")
public class ServoPositionFinder extends LinearOpMode {
    private static final String[] NAMES = {
            HardwareNames.DOOR_SERVO,
            HardwareNames.FLOWER_LEFT_SERVO,
            HardwareNames.FLOWER_RIGHT_SERVO,
    };

    @Override
    public void runOpMode() {
        Servo[] servos = new Servo[NAMES.length];
        double[] positions = new double[NAMES.length];
        for (int i = 0; i < NAMES.length; i++) {
            servos[i] = hardwareMap.get(Servo.class, NAMES[i]);
            positions[i] = 0.5;
        }
        int selected = 0;

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpadRightWasPressed()) selected = (selected + 1) % NAMES.length;
            if (gamepad1.dpadLeftWasPressed()) selected = (selected + NAMES.length - 1) % NAMES.length;

            double change = 0;
            if (gamepad1.dpadUpWasPressed()) change = 0.01;
            if (gamepad1.dpadDownWasPressed()) change = -0.01;
            if (gamepad1.rightBumperWasPressed()) change = 0.05;
            if (gamepad1.leftBumperWasPressed()) change = -0.05;
            if (change != 0) {
                positions[selected] = Math.max(0, Math.min(1, positions[selected] + change));
                servos[selected].setPosition(positions[selected]);
            }

            for (int i = 0; i < NAMES.length; i++) {
                telemetry.addData((i == selected ? "> " : "  ") + NAMES[i], "%.2f", positions[i]);
            }
            telemetry.addLine("The right flower servo is reversed in FlowerIntake, so its number here is 1 minus the one to use.");
            telemetry.update();
        }
    }
}
