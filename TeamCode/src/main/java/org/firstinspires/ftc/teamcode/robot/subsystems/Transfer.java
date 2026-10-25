package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.hardware.CachedMotor;
import org.firstinspires.ftc.teamcode.robot.hardware.HardwareNames;

/**
 * The transfer motor. It drives the door end of the rail (slots 1-2); Intake
 * drives the intake end. Positive power moves balls toward the door.
 *
 * No update(): each method writes the motor right away.
 */
@Configurable
public class Transfer {
    // TODO(3): Storage Test, Cross on: the door end must move balls toward the door.
    public static boolean REVERSED = false;

    // TODO(3): Lower it if balls jam or bounce.
    public static double COLLECT_POWER = 1.0;
    // TODO(3): Lowest power that keeps a ball seated on the closed door without the motor straining.
    /** Once a ball sits against the closed door, push gently so the motor doesn't stall. */
    public static double HOLD_POWER = 0.15;
    // TODO(3): Storage Test, hold Circle with balls loaded: lower it (toward 0) along with Intake.REVERSE_POWER if balls fly out of the intake.
    /** Unjam power, negative: balls move away from the door. */
    public static double REVERSE_POWER = -1.0;

    private final CachedMotor motor;
    private String state = "stopped";

    public Transfer(HardwareMap hardwareMap) {
        DcMotorEx transferMotor = hardwareMap.get(DcMotorEx.class, HardwareNames.TRANSFER_MOTOR);
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor = new CachedMotor(transferMotor);
    }

    /** The motor stays FORWARD. REVERSED is applied here, so flipping it in Panels works live. */
    private void setPower(double power) {
        motor.setPower(REVERSED ? -power : power);
    }

    /** Normal collecting: pull balls toward the door, and hold the front ball against it. */
    public void collect(boolean slot1Occupied) {
        setPower(slot1Occupied ? HOLD_POWER : COLLECT_POWER);
        state = slot1Occupied ? "holding" : "collecting";
    }

    /** Push balls through the open door into the shooter. */
    public void feed(double power) {
        setPower(power);
        state = "feeding";
    }

    public void reverse() {
        setPower(REVERSE_POWER);
        state = "reversing";
    }

    public void stop() {
        setPower(0);
        state = "stopped";
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Transfer", state);
    }
}
