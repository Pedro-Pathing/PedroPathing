package com.pedropathing.algorithm;

import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.FollowState;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.PathTracker;
import com.pedropathing.paths.SimplePath;
import com.pedropathing.paths.curves.Line;
import com.pedropathing.paths.interpolator.Interpolator;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class ForesightTest {
    static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                c.translationalController.set(Controller.pid(0.16,0,0.01));
                c.headingController.set(Controller.pid(2.0,0,0.1));
                c.linearBrakeCoefficients.set(Matrix.diag(0.139333365, 0.139333365));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.000210842, 0.000210842));
                c.maxAchievableForwardVelocity.set(88.036);
                c.maxAchievableStrafeVelocity.set(71.881);
                c.maxAchievableForwardDeceleration.set(30.3333);
                c.maxAchievableStrafeDeceleration.set(62.58098);
            }
    );

    @Test
    public void hold_atTargetProducesZeroDrive() {
        Foresight f = new Foresight(foresightConfig);
        Pose pose = Pose.zero();
        MotionState ms = MotionState.ofVelocity(pose, Velocity.zero());
        // use a very short non-zero line to avoid Line normalizing a zero vector
        DrivePowers dp = f.hold(pose, new FollowState(ms, new PathTracker(new SimplePath(new Line(Pose.zero(), new Pose(0.01, 0, 0)), Interpolator.tangent)), 0.02));
        assertNotNull(dp);
        assertEquals(0.0, dp.forward(), 1e-6);
        assertEquals(0.0, dp.strafe(), 1e-6);
        assertEquals(0.0, dp.turn(), 1e-6);
    }

    @Test
    public void hold_withTranslationErrorProducesNonZeroDrive() {
        Foresight f = new Foresight(foresightConfig);
        Pose target = new Pose(1.0, 0.0, 0.0);
        MotionState ms = MotionState.ofVelocity(Pose.zero(), Velocity.zero());
        DrivePowers dp = f.hold(target, new FollowState(ms, new PathTracker(new SimplePath(new Line(Pose.zero(), target), Interpolator.tangent)), 0.02));
        assertNotNull(dp);
        // Expect some forward power to correct the 1.0 unit translational error.
        assertTrue(Math.abs(dp.forward()) > 1e-6 || Math.abs(dp.strafe()) > 1e-6);
    }

    @Test
    public void calculate_basicForwardPathProducesForwardDrive() {
        Foresight f = new Foresight(foresightConfig);
        Line line = new Line(Pose.zero(), new Pose(10.0, 0.0, 0.0));
        SimplePath path = new SimplePath(line, Interpolator.tangent);
        PathTracker tracker = new PathTracker(path);

        MotionState ms = MotionState.ofVelocity(Pose.zero(), Velocity.zero());
        FollowState state = new FollowState(ms, tracker, 0.02);

        DrivePowers dp = f.calculate(state);
        assertNotNull(dp);
        // drive should be commanding forward motion when stationary at the start of a forward path
        assertTrue(dp.forward() >= 0.0);
    }
}

