package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;

// Our tuner list. The Pedro tuning menu finds every static, no-arg @Tuner method
// that returns Procedure in any class under org.firstinspires.ftc.teamcode.
// Tuning.java is upstream's empty class, so we keep our list here instead.
public class TeamTuning {
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    // TODO(2): After pasting the Mecanum and Pinpoint tuner output into Constants, add ForesightTuner here. Add Tests after pasting the Foresight output too.
    // Imports to add:
    //   import com.pedropathing.algorithm.Foresight;
    //   import com.pedropathing.revhub.drivetrains.Mecanum;
    //   import com.pedropathing.revhub.localizers.PinpointLocalizer;
    //   import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
    //   import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;
    // Methods to add (needs Constants.drivetrainConfig and Constants.localizerConfig):
    //   @Tuner
    //   public static Procedure foresightTuner() {
    //       return new ForesightTuner(
    //               hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
    //               hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig));
    //   }
    // TODO(2): Then add DriveModelTuner the same way (code below). It measures the numbers agateflow/DriveModel needs.
    //   import org.firstinspires.ftc.teamcode.pedro.procedures.DriveModelTuner;
    //   @Tuner
    //   public static Procedure driveModelTuner() {
    //       return new DriveModelTuner(
    //               hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
    //               hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig));
    //   }
    // Method to add after Constants.foresightConfig() exists too:
    //   @Tuner
    //   public static Procedure tests() {
    //       return new Tests(
    //               hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
    //               hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
    //               () -> new Foresight(Constants.foresightConfig()));
    //   }
}
