package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.util.Angles;

/**
 * The blue half of the field is the red half spun 180° around the center,
 * so every pose is written once for red and flipped here for blue.
 */
public enum Alliance {
    RED,
    BLUE;

    public Pose fromRed(Pose redPose) {
        if (this == RED) return redPose;
        return new Pose(
                Field.SIZE - redPose.x(),
                Field.SIZE - redPose.y(),
                Angles.wrapRadians(redPose.heading() + Math.PI));
    }

    /** Field heading the driver looks along from their ALLIANCE AREA. */
    public double driverHeading() {
        return this == RED ? 0 : Math.PI;
    }

    public Alliance other() {
        return this == RED ? BLUE : RED;
    }
}
