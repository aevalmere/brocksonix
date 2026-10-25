package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    // TODO(2): Run the tuners in TeamTuning.java (MecanumTuner, PinpointTuner, then ForesightTuner) and paste the results here.
    // TODO(2): Robot forward (heading 0) must be the intake end. Corner poses and aiming depend on it.
    // TODO(2): Run the Pedro Tests procedure (Localization, Line, Curve).

    // Starting values, so Drive Test runs before any tuning. The MecanumTuner output (drivetrainConfig)
    // and the PinpointTuner output (localizerConfig) replace these two as printed.
    // TODO(2): Motor directions are a guess (left side reversed). Check them with Drive Test's D-pad, or run MecanumTuner.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // TODO(2): Pinpoint pod offsets from CAD are in ROBOT.md (X pod +2.54 in. left, Y pod -1.55 in. forward). Confirm with PinpointTuner.
    // TODO(2): Pod directions are a guess. Check them with Drive Test (push 48 in. forward and left), or run PinpointTuner.
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(2.54);
        c.yPodOffset.set(-1.55);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // The ForesightTuner output goes inside a method instead of a field, so every Follower gets a
    // new config. While a path runs, its speed limit is written into the config. If an OpMode stops
    // in the middle of that path (or crashes), a shared config would keep the limit, and every later
    // OpMode would drive slow until the app restarts. Four steps:
    //   1. Type this line:  public static ForesightConfig foresightConfig() {
    //   2. Under it, paste everything the tuner printed, as it is.
    //   3. In the first pasted line, delete "public static " so it starts "ForesightConfig foresightConfig = new ForesightConfig(".
    //   4. Under the last pasted line (the tuner's ");"), type  return foresightConfig;  and then  }
    // The finished method looks like this. You type the first line and the last two; everything
    // between is the tuner's lines as printed, minus "public static " on the first of them:
    //
    //   public static ForesightConfig foresightConfig() {
    //   ForesightConfig foresightConfig = new ForesightConfig(
    //               c -> {
    //                   Controller primaryTranslationalForward = Controller.proportional(0.123);
    //                   (all the other lines the tuner printed)
    //                   c.naturalStrafeDeceleration.set(45.6);
    //               }
    //       );
    //       return foresightConfig;
    //   }
    //
    // Imports it needs: com.pedropathing.algorithm.ForesightConfig, com.pedropathing.controllers.Controller,
    // com.pedropathing.math.Matrix and com.pedropathing.math.Vector2D.
    // Everything else that builds a Foresight (like Tests in TeamTuning) calls foresightConfig() too.

    public static Follower create(HardwareMap h) {
        // return new Follower(new PinpointLocalizer(h, localizerConfig), new Mecanum(h, drivetrainConfig), new Foresight(foresightConfig()));
        return null;
    }
}
