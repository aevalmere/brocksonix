package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    // TODO(2): Pinpoint pod offsets from CAD are in ROBOT.md (X pod +2.54 in. left, Y pod -1.55 in. forward). Confirm with PinpointTuner.
    // TODO(2): Run the tuners in TeamTuning.java (MecanumTuner, PinpointTuner, then ForesightTuner) and paste the results here.
    // TODO(2): Robot forward (heading 0) must be the intake end. Corner poses and aiming depend on it.
    // TODO(2): Run the Pedro Tests procedure. Push the robot 48 in. by hand and check the pose moves 48 in.

    // Paste the MecanumTuner output (drivetrainConfig) and the PinpointTuner output (localizerConfig) here as they are.

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
