package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.Servo;

/** Only sends a position when it changes. Every hardware write costs loop time. */
public class CachedServo {
    private final Servo servo;
    private double last = Double.NaN;

    public CachedServo(Servo servo) {
        this.servo = servo;
    }

    public void setPosition(double position) {
        if (position != last) {
            servo.setPosition(position);
            last = position;
        }
    }

    /**
     * The servo itself, for what this class doesn't wrap. A call through it (setDirection,
     * scaleRange, a new PWM range) changes where a position lands, so the cache is cleared:
     * the next setPosition() always goes out, even if it repeats the last one.
     */
    public Servo raw() {
        last = Double.NaN;
        return servo;
    }
}
