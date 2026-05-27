package com.pedropathing.localization;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Velocity;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class MotionStateTest {
    @Test
    public void factoriesProduceConsistentTwistAndVelocity() {
        Pose pose = new Pose(1.0, 2.0, Math.PI / 4);
        Velocity v = new Velocity(1.0, 0.0, 0.5);

        MotionState ms = MotionState.ofVelocity(pose, v);
        assertEquals(v.vx, ms.velocity().vx, 1e-9);
        assertEquals(v.vy, ms.velocity().vy, 1e-9);
        // twist should be consistent with velocity at pose heading
        Twist t = ms.twist();
        assertEquals(3, t.toVector().size());

        MotionState ms2 = MotionState.ofTwist(pose, t);
        assertEquals(ms2.twist().vx(), t.vx(), 1e-9);
    }
}

