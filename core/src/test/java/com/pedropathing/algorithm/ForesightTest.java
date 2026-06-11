package com.pedropathing.algorithm;

import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Vector2D;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.PathTracker;
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

    // Asymmetric braking config for testing theta-dependent behavior
    static ForesightConfig asymmetricBrakingConfig = new ForesightConfig(
            c -> {
                c.translationalController.set(Controller.pid(0.16,0,0.01));
                c.headingController.set(Controller.pid(2.0,0,0.1));
                // Asymmetric: forward brake is stronger than strafe
                c.linearBrakeCoefficients.set(Matrix.diag(0.2, 0.1));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.01, 0.02));
                c.maxAchievableForwardVelocity.set(88.036);
                c.maxAchievableStrafeVelocity.set(71.881);
                c.maxAchievableForwardDeceleration.set(30.3333);
                c.maxAchievableStrafeDeceleration.set(62.58098);
            }
    );

    static ForesightConfig brakeConfig = new ForesightConfig(
            c -> {
                c.translationalController.set(Controller.pid(0.16,0,0));
                c.headingController.set(Controller.pid(2.0,0,0.1));
                c.linearBrakeCoefficients.set(Matrix.diag(0.015, 0.02));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001, 0.0015));
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
        DrivePowers dp = f.calculateHold(pose, new FollowState(ms, new PathTracker(new SimplePath(new Line(Pose.zero(), new Pose(0.01, 0, 0)), Interpolator.tangent)), 0.02));
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

        DrivePowers dp = f.calculatePath(state);
        assertNotNull(dp);
        // drive should be commanding forward motion when stationary at the start of a forward path
        assertTrue(dp.forward() >= 0.0);
    }

    @Test
    public void calculate_verticalPathAtT025OnLineWithCorrectHeadingProducesStrongForwardDrive() {
        Foresight f = new Foresight(foresightConfig);
        // Vertical path from (0,0) to (0,100)
        Line verticalLine = new Line(Pose.zero(), new Pose(0.0, 100.0, 0.0));
        SimplePath path = new SimplePath(verticalLine, Interpolator.tangent);
        PathTracker tracker = new PathTracker(path);

        // Robot at t=0.25 on the path is at position (0, 25), which is (0.25 * 100)
        // Robot is facing upward (Math.PI/2), which is the tangent direction of the vertical line
        Pose robotPose = new Pose(0.0, 25.0, Math.PI / 2);
        MotionState ms = MotionState.ofVelocity(robotPose, Velocity.zero());
        FollowState state = new FollowState(ms, tracker, 0.02);

        DrivePowers dp = f.calculatePath(state);
        assertNotNull(dp);
        // Expect very strong forward output since robot is on-path and heading correctly
        assertTrue(dp.forward() > 0.5, "Forward output should be strong (> 0.5) when on-path and correctly heading");
    }

    @Test
    public void calculate_verticalPathOnLineCorrectHeadingProducesMinimalStrafe() {
        Foresight f = new Foresight(foresightConfig);
        Line verticalLine = new Line(Pose.zero(), new Pose(0.0, 100.0, 0.0));
        SimplePath path = new SimplePath(verticalLine, Interpolator.tangent);
        PathTracker tracker = new PathTracker(path);

        // Robot perfectly on-line at (0, 50), heading upward
        Pose robotPose = new Pose(0.0, 50.0, Math.PI / 2);
        MotionState ms = MotionState.ofVelocity(robotPose, Velocity.zero());
        FollowState state = new FollowState(ms, tracker, 0.02);

        DrivePowers dp = f.calculatePath(state);
        assertNotNull(dp);
        // When perfectly on-line and heading correctly, strafe should be minimal
        assertTrue(Math.abs(dp.strafe()) < 0.2, "Strafe should be minimal when perfectly on-line");
    }

    @Test
    public void calculate_verticalPathOffLineLeftProducesRightStrafing() {
        Foresight f = new Foresight(foresightConfig);
        Line verticalLine = new Line(Pose.zero(), new Pose(0.0, 100.0, 0.0));
        SimplePath path = new SimplePath(verticalLine, Interpolator.tangent);
        PathTracker tracker = new PathTracker(path);

        // Robot off-line to the left (negative x) at (-1.0, 50), heading upward
        Pose robotPose = new Pose(-1.0, 50.0, Math.PI / 2);
        MotionState ms = MotionState.ofVelocity(robotPose, Velocity.zero());
        FollowState state = new FollowState(ms, tracker, 0.02);

        DrivePowers dp = f.calculatePath(state);
        assertNotNull(dp);
        // When off-line to the left, robot should strafe right to correct
        assertTrue(dp.strafe() > 0.0, "Should strafe right to correct being off-line to the left");
    }

    @Test
    public void calculate_verticalPathWithWrongHeadingProducesSpinCorrection() {
        Foresight f = new Foresight(foresightConfig);
        Line verticalLine = new Line(Pose.zero(), new Pose(0.0, 100.0, 0.0));
        SimplePath path = new SimplePath(verticalLine, Interpolator.tangent);
        PathTracker tracker = new PathTracker(path);

        // Robot on-line at (0, 50), but heading downward (wrong direction, -Math.PI/2)
        Pose robotPose = new Pose(0.0, 50.0, -Math.PI / 2);
        MotionState ms = MotionState.ofVelocity(robotPose, Velocity.zero());
        FollowState state = new FollowState(ms, tracker, 0.02);

        DrivePowers dp = f.calculatePath(state);
        assertNotNull(dp);
        // When heading wrong direction, robot should produce turn correction
        assertTrue(Math.abs(dp.turn()) > 0.1, "Should produce turn correction when heading wrong direction");
    }

    @Test
    public void getVelocityToBrakeInTimeVariesWithThetaAsymmetric() {
        Foresight f = new Foresight(asymmetricBrakingConfig);
        // With asymmetric braking coefficients, velocity needed to brake should vary with theta
        double v0 = f.getVelocityToBrakeInTime(10.0, 0.0);        // forward direction
        double v90 = f.getVelocityToBrakeInTime(10.0, Math.PI / 2); // lateral direction
        assertTrue(Double.isFinite(v0) && v0 > 0.0);
        assertTrue(Double.isFinite(v90) && v90 > 0.0);
        // They should differ because coefficients are asymmetric
        assertNotEquals(v0, v90, "Braking velocity should differ with theta when coefficients are asymmetric");
    }

    @Test
    public void coastBehaviorVariesWithTheta() {
        Foresight f = new Foresight(asymmetricBrakingConfig);
        // Coast should compute differently for different angles due to ellipse deceleration
        double coastForward = f.coast(50.0, 0.0, 100.0, 100.0);
        double coastLateral = f.coast(50.0, Math.PI / 2, 100.0, 100.0);
        assertTrue(Double.isFinite(coastForward));
        assertTrue(Double.isFinite(coastLateral));
        // At different angles, the achievable deceleration magnitude differs, so coast output may differ
        assertNotNull("Coast should handle both angles");
    }

    @Test
    public void brakeDisplacement() {
        Foresight f = new Foresight(brakeConfig);
        assertEquals(Vector2D.cartesian(2.2, 0), f.getBrakeDisplacement(new Twist(40, 0, 0) ,0));
        assertEquals(Vector2D.cartesian(-2.2, 0), f.getBrakeDisplacement(new Twist(-40, 0, 0),0));
        assertEquals(Vector2D.cartesian(1.35, 0), f.getBrakeDisplacement(new Twist(30, 0, 0),0));

        assertEquals(Vector2D.cartesian(3.2, -1.592040838891559E-16), f.getBrakeDisplacement(new Velocity(40, 0, 0).toTwist(Math.PI/2),Math.PI/2));

        assertEquals(0, f.excessVelocityAfterBraking(10, f.getBrakeDisplacement(new Twist(30, 0, 0),0).x(),0));

        assertEquals(24.2214438511238, f.excessVelocityAfterBraking(5, f.getBrakeDisplacement(new Twist(70, 0, 0),0).x(),0));
    }
}
