package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
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

    // Run it after the Mecanum and Pinpoint tuner output is pasted into Constants.
    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig));
    }

    // TODO(2): After pasting the Foresight output into Constants (as foresightConfig()), add Tests here (code at the bottom).
    // TODO(13): Add DriveModelTuner the same way as ForesightTuner (code below). It measures the numbers agateflow/DriveModel needs.
    //   import org.firstinspires.ftc.teamcode.pedro.procedures.DriveModelTuner;
    //   @Tuner
    //   public static Procedure driveModelTuner() {
    //       return new DriveModelTuner(
    //               hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
    //               hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig));
    //   }
    // Tests, once Constants.foresightConfig() exists:
    //   import com.pedropathing.algorithm.Foresight;
    //   import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;
    //   @Tuner
    //   public static Procedure tests() {
    //       return new Tests(
    //               hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
    //               hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
    //               () -> new Foresight(Constants.foresightConfig()));
    //   }
}
