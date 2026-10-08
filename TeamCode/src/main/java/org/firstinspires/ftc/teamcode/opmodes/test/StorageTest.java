package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.hardware.BulkReads;
import org.firstinspires.ftc.teamcode.robot.subsystems.Door;
import org.firstinspires.ftc.teamcode.robot.subsystems.Rail;
import org.firstinspires.ftc.teamcode.robot.subsystems.Storage;

/**
 * Checks the break beams and the ball count. Block each beam by hand first,
 * then run the rail and feed real balls in. The door stays closed the whole
 * time, so balls stack up against it like in a match (find Door.CLOSED_POSITION first).
 *
 * Cross: rail collect on/off. Circle (hold): reverse.
 */
@TeleOp(name = "Storage Test", group = "Test")
public class StorageTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Telemetry screen = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        BulkReads bulkReads = new BulkReads(hardwareMap);
        Storage storage = new Storage(hardwareMap);
        Rail rail = new Rail(hardwareMap);
        Door door = new Door(hardwareMap);
        boolean collecting = false;

        waitForStart();
        while (opModeIsActive()) {
            bulkReads.clear();
            storage.update();

            if (gamepad1.crossWasPressed()) collecting = !collecting;
            if (gamepad1.circle) rail.reverse();
            else if (collecting) rail.collect(storage.slot1Occupied(), storage.isFull());
            else rail.stop();

            // Written every loop, so a Panels edit to CLOSED_POSITION shows up right away.
            door.close();
            door.update();

            screen.addData("Collecting (Cross)", collecting);
            storage.addTelemetry(screen);
            rail.addTelemetry(screen);
            door.addTelemetry(screen);
            screen.update();
        }
    }
}
