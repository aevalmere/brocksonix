package org.firstinspires.ftc.teamcode.util;

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

    public Servo raw() {
        return servo;
    }
}
