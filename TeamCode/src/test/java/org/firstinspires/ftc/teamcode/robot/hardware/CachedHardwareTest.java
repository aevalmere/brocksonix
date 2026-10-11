package org.firstinspires.ftc.teamcode.robot.hardware;

import static org.junit.Assert.assertEquals;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.junit.Test;

import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.List;

/** The write caches skip repeats, but never a stop, and never the first write after raw() handed out the device. */
public class CachedHardwareTest {
    /** Every value the fake device was sent. */
    private final List<Double> sent = new ArrayList<>();

    /** A fake device that writes down every value sent to setter. Nothing else on it may be called. */
    private <T> T fake(Class<T> type, final String setter) {
        Object device = Proxy.newProxyInstance(type.getClassLoader(), new Class<?>[]{type},
                (proxy, method, args) -> {
                    if (!method.getName().equals(setter)) throw new UnsupportedOperationException(method.getName());
                    sent.add((Double) args[0]);
                    return null;
                });
        return type.cast(device);
    }

    @Test
    public void servoSkipsARepeatUntilRawIsUsed() {
        CachedServo servo = new CachedServo(fake(Servo.class, "setPosition"));
        servo.setPosition(0.3);
        servo.setPosition(0.3);
        assertEquals(1, sent.size());
        // A setDirection or scaleRange through raw() moves where 0.3 lands, so the repeat must go out.
        servo.raw();
        servo.setPosition(0.3);
        assertEquals(2, sent.size());
    }

    @Test
    public void motorSkipsSmallChangesButNeverAStop() {
        CachedMotor motor = new CachedMotor(fake(DcMotorEx.class, "setPower"), 0.01);
        motor.setPower(0.5);
        motor.setPower(0.505);
        assertEquals(1, sent.size());
        motor.setPower(0.005);
        // Only 0.005 away, under the 0.01 step, but it is a stop.
        motor.setPower(0);
        assertEquals(3, sent.size());
        assertEquals(0, sent.get(2), 0);
    }

    @Test
    public void motorSendsARepeatAfterRawIsUsed() {
        CachedMotor motor = new CachedMotor(fake(DcMotorEx.class, "setPower"), 0.01);
        motor.setPower(0.5);
        // setMode(STOP_AND_RESET_ENCODER) through raw() would set the power to 0 behind the cache.
        motor.raw();
        motor.setPower(0.5);
        assertEquals(2, sent.size());
    }

    @Test
    public void crServoSendsARepeatAfterRawIsUsed() {
        CachedCRServo servo = new CachedCRServo(fake(CRServo.class, "setPower"));
        servo.setPower(0.4);
        servo.setPower(0.4);
        assertEquals(1, sent.size());
        servo.raw();
        servo.setPower(0.4);
        assertEquals(2, sent.size());
    }
}
