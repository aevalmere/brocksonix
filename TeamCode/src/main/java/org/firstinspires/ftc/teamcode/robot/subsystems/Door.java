package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.hardware.CachedServo;
import org.firstinspires.ftc.teamcode.robot.hardware.HardwareNames;

/** Servo gate between slot 1 and the shooter. Closed holds balls back, open lets them feed. */
@Configurable
public class Door {
    /** The servo was zeroed with the door open. */
    public static double OPEN_POSITION = 1;
    public static double CLOSED_POSITION = 0.76;
    /** Time for the servo to swing fully open. Feeding waits for this. */
    public static double OPEN_TIME_MS = 55;
    /** Time for the servo to swing fully closed. The rail waits for this after a burst. */
    public static double CLOSE_TIME_MS = 55;

    private final CachedServo servo;
    private final ElapsedTime sinceOpened = new ElapsedTime();
    private final ElapsedTime sinceClosed = new ElapsedTime();
    private boolean open = false;

    public Door(HardwareMap hardwareMap) {
        servo = new CachedServo(hardwareMap.get(Servo.class, HardwareNames.DOOR_SERVO));
    }

    public void open() {
        if (!open) sinceOpened.reset();
        open = true;
    }

    public void close() {
        if (open) sinceClosed.reset();
        open = false;
    }

    public boolean isFullyOpen() {
        return open && sinceOpened.milliseconds() >= OPEN_TIME_MS;
    }

    public boolean isFullyClosed() {
        return !open && sinceClosed.milliseconds() >= CLOSE_TIME_MS;
    }

    public void update() {
        servo.setPosition(open ? OPEN_POSITION : CLOSED_POSITION);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Door", open ? (isFullyOpen() ? "open" : "opening") : (isFullyClosed() ? "closed" : "closing"));
    }
}
