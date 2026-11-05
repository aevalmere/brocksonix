package org.firstinspires.ftc.teamcode.sim;

import org.horizon36596.simloop.fakehardware.FakeGoBildaPinpoint;

/**
 * SimLoop's Pinpoint, ticked by RobotSim right after the drive plant moves it
 * instead of before (SimLoop ticks every registered device first, then runs the
 * plants). Its velocity is the motion since its last tick, so this way the
 * follower reads this tick's velocity with this tick's pose, not one tick late.
 */
final class LockstepPinpoint extends FakeGoBildaPinpoint {
    /** SimLoop's own tick: skipped, see the class comment. */
    @Override
    public void update(double deltaTime) {
    }

    /** The device's tick, once the chassis has moved this tick. */
    void advance(double deltaTime) {
        super.update(deltaTime);
    }
}
