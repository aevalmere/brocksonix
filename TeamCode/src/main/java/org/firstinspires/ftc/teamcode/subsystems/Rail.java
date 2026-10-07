package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.HardwareNames;
import org.firstinspires.ftc.teamcode.util.CachedMotor;

/**
 * The storage rail. The intake motor drives the intake end (slots 3-4) and
 * the transfer motor drives the door end (slots 1-2). Positive power moves
 * balls toward the door.
 */
@Configurable
public class Rail {
    // TODO(3): Storage Test, Cross on: both ends must move balls toward the door.
    public static boolean INTAKE_REVERSED = false;
    public static boolean TRANSFER_REVERSED = false;

    // TODO(3): Lower the collect powers if balls jam or bounce.
    public static double INTAKE_POWER = 1.0;
    public static double TRANSFER_INTAKE_POWER = 1.0;
    // TODO(3): Lowest power that keeps a ball seated on the closed door without the motor straining.
    /** Once a ball sits against the closed door, push gently so the motor doesn't stall. */
    public static double TRANSFER_HOLD_POWER = 0.15;
    public static double REVERSE_POWER = -1.0;

    private final CachedMotor intake, transfer;
    private String state = "stopped";

    public Rail(HardwareMap hardwareMap) {
        intake = motor(hardwareMap, HardwareNames.INTAKE_MOTOR);
        transfer = motor(hardwareMap, HardwareNames.TRANSFER_MOTOR);
    }

    /** The motors stay FORWARD. The REVERSED flags are applied in setIntake/setTransfer so they work live in Panels. */
    private static CachedMotor motor(HardwareMap hardwareMap, String name) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return new CachedMotor(motor);
    }

    private void setIntake(double power) {
        intake.setPower(INTAKE_REVERSED ? -power : power);
    }

    private void setTransfer(double power) {
        transfer.setPower(TRANSFER_REVERSED ? -power : power);
    }

    /** Normal collecting: pull balls in until full, and hold the front ball against the door. */
    public void collect(boolean slot1Occupied, boolean full) {
        setIntake(full ? 0 : INTAKE_POWER);
        setTransfer(slot1Occupied ? TRANSFER_HOLD_POWER : TRANSFER_INTAKE_POWER);
        state = full ? "full" : "collecting";
    }

    /** Push balls through the open door into the shooter. */
    public void feed(double power) {
        setIntake(power);
        setTransfer(power);
        state = "feeding";
    }

    public void reverse() {
        setIntake(REVERSE_POWER);
        setTransfer(REVERSE_POWER);
        state = "reversing";
    }

    public void stop() {
        setIntake(0);
        setTransfer(0);
        state = "stopped";
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Rail", state);
    }
}
