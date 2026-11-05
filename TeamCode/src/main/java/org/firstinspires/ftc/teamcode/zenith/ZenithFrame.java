package org.firstinspires.ftc.teamcode.zenith;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Field;

/**
 * Zenith's files put (0, 0) at the center of the field. Our Pedro field puts it in the
 * corner between the red wall and the audience wall (field/Field.java). Everything else
 * is the same in both: +x toward blue, +y away from the audience, inches, and headings
 * in radians from +x, counter-clockwise. So a pose from a file only moves by half the field.
 *
 * Zenith's own field file is drawn on the manual's 144 in. field. Our copy
 * (autos/field/biobuzz.field.json) is converted to 141.5 (Field.SIZE), walls at +-70.75, so
 * the two centers are the same point. That keeps blue's side (Alliance.fromRed, a spin about
 * our center) exactly the mirror image Zenith draws for blue (a spin about its center).
 */
public final class ZenithFrame {
    private ZenithFrame() {}

    /** Where Zenith's (0, 0), the field center, is on our field, in both x and y. */
    public static final double CENTER = Field.SIZE / 2;

    /** A pose from a Zenith file on our field. The heading doesn't change. */
    public static Pose toPedro(Pose zenithPose) {
        return new Pose(zenithPose.x() + CENTER, zenithPose.y() + CENTER, zenithPose.heading());
    }
}
