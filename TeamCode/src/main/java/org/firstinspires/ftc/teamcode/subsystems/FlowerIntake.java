package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.HardwareNames;
import org.firstinspires.ftc.teamcode.util.CachedServo;

/**
 * Two mirrored servos that lower an arm into a FLOWER's Retrieval Opening.
 * The right servo is reversed, so both take the same position number.
 */
@Configurable
public class FlowerIntake {
    // TODO(4): Find up and down with Servo Position Finder, on the left servo.
    public static double UP_POSITION = 0.2;
    // TODO(4): Check it reaches into a real FLOWER's Retrieval Opening and pulls POLLEN out.
    public static double DOWN_POSITION = 0.7;
    // TODO(4): LB in Main TeleOp must move both servos the same way. Flip if they fight.
    public static boolean RIGHT_REVERSED = true;

    private final CachedServo left, right;
    private boolean down = false;

    public FlowerIntake(HardwareMap hardwareMap) {
        left = new CachedServo(hardwareMap.get(Servo.class, HardwareNames.FLOWER_LEFT_SERVO));
        // Stays FORWARD. RIGHT_REVERSED is applied in update() so it works live in Panels.
        right = new CachedServo(hardwareMap.get(Servo.class, HardwareNames.FLOWER_RIGHT_SERVO));
    }

    public void toggle() {
        down = !down;
    }

    public void setDown(boolean down) {
        this.down = down;
    }

    public boolean isDown() {
        return down;
    }

    public void update() {
        double position = down ? DOWN_POSITION : UP_POSITION;
        left.setPosition(position);
        // A reversed servo mirrors the position around the middle, so send 1 - position.
        right.setPosition(RIGHT_REVERSED ? 1 - position : position);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Flower intake", down ? "down" : "up");
    }
}
