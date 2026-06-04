package com.pedropathing.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.junit.jupiter.api.Test;

/**
 * Tests for {@link FusionLocalizer}, focused on the three correctness fixes:
 * <ol>
 *     <li>translation is propagated in the fused frame (vision heading corrections are honored),</li>
 *     <li>process noise scales with distance travelled (loop-rate invariant, no stationary inflation),</li>
 *     <li>degenerate measurement variances are floored instead of crashing.</li>
 * </ol>
 */
class FusionLocalizerTest {
    private static final double EPS = 1e-6;

    /** A {@link Localizer} whose pose/velocity are set directly by the test. */
    private static class FakeLocalizer implements Localizer {
        private Pose pose = new Pose();
        private Pose velocity = new Pose();
        // Distinct constants so delegation through FusionLocalizer is observable.
        double imuHeading = 0;
        double angularVelocity = 0;

        void set(Pose pose) { this.pose = pose; }
        void setVelocity(Pose velocity) { this.velocity = velocity; }

        @Override public void update() { }
        @Override public Pose getPose() { return pose; }
        @Override public Pose getVelocity() { return velocity; }
        @Override public Vector getVelocityVector() { return velocity.getAsVector(); }
        @Override public void setStartPose(Pose setStart) { pose = setStart; }
        @Override public void setPose(Pose setPose) { pose = setPose; }
        @Override public double getTotalHeading() { return pose.getHeading(); }
        @Override public double getForwardMultiplier() { return 2; }
        @Override public double getLateralMultiplier() { return 3; }
        @Override public double getTurningMultiplier() { return 4; }
        @Override public void resetIMU() { }
        @Override public double getIMUHeading() { return imuHeading; }
        @Override public boolean isNAN() { return false; }
        @Override public double getAngularVelocity() { return angularVelocity; }
    }

    /** A {@link FusionLocalizer} with a deterministic, test-driven clock. */
    private static class TestableFusion extends FusionLocalizer {
        long clock = 0;
        TestableFusion(Localizer dr, Pose initialCov, Pose processVar, Pose measVar, int buffer) {
            super(dr, initialCov, processVar, measVar, buffer);
        }
        @Override protected long currentTimeNanos() { return clock; }
    }

    /** Advances the odometry to {@code odomPose} and runs one fused update at time {@code t}. */
    private static void step(TestableFusion fused, FakeLocalizer odom, long t, Pose odomPose) {
        odom.set(odomPose);
        fused.clock = t;
        fused.update();
    }

    private static void assertPoseEquals(Pose expected, Pose actual, double tol) {
        assertEquals(expected.getX(), actual.getX(), tol, "x");
        assertEquals(expected.getY(), actual.getY(), tol, "y");
        assertEquals(expected.getHeading(), actual.getHeading(), tol, "heading");
    }

    // ---- #1: frame correctness --------------------------------------------------------------

    /** With no vision, the fused pose is exactly the odometry pose (SE(2) increments composed onto the same start). */
    @Test
    void noVision_fusedTracksOdometryThroughAnArc() {
        FakeLocalizer odom = new FakeLocalizer();
        TestableFusion fused = new TestableFusion(odom,
                new Pose(1, 1, 1), new Pose(1, 1, 1), new Pose(1, 1, 1), 1000);
        fused.setStartPose(new Pose(0, 0, 0));

        Pose[] path = {
                new Pose(2, 0, 0),
                new Pose(4, 1, Math.PI / 6),
                new Pose(5, 3, Math.PI / 3),
                new Pose(5, 6, Math.PI / 2),
        };
        long t = 1;
        for (Pose p : path) {
            step(fused, odom, t++, p);
            assertPoseEquals(p, fused.getPose(), 1e-9);
        }
    }

    /**
     * The key fix: after a vision heading correction, subsequent forward odometry motion must move
     * the fused pose along the <em>corrected</em> heading, not the (uncorrected) odometry heading.
     */
    @Test
    void visionHeadingCorrection_redirectsSubsequentTranslation() {
        FakeLocalizer odom = new FakeLocalizer();
        // Huge initial covariance + tiny measurement variance => the correction snaps to the measurement.
        TestableFusion fused = new TestableFusion(odom,
                new Pose(1e6, 1e6, 1e6), new Pose(1, 1, 1), new Pose(EPS, EPS, EPS), 1000);
        fused.setStartPose(new Pose(0, 0, 0));

        // Two stationary updates so timestamp 1 is a valid (interior) measurement time.
        step(fused, odom, 1, new Pose(0, 0, 0));
        step(fused, odom, 2, new Pose(0, 0, 0));

        // Correct heading to +90 degrees at the origin.
        fused.addMeasurement(new Pose(0, 0, Math.PI / 2), 1, new Pose(EPS, EPS, EPS));
        assertEquals(Math.PI / 2, fused.getPose().getHeading(), 1e-3, "heading should snap to measurement");

        // Now drive 10" forward in the odometry frame (heading still 0 there).
        step(fused, odom, 3, new Pose(10, 0, 0));

        // Forward motion along the corrected 90-degree heading => +y, not +x.
        assertEquals(0, fused.getPose().getX(), 1e-2, "x should not move along the stale odom heading");
        assertEquals(10, fused.getPose().getY(), 1e-2, "translation should follow the corrected heading");
    }

    // ---- #2: distance-scaled process noise --------------------------------------------------

    /**
     * Stationary updates must not inflate covariance: a measurement applied after many no-motion
     * loops must produce the same correction as one applied immediately (loop-rate invariance).
     */
    @Test
    void stationaryUpdates_doNotInflateCovariance() {
        Pose measurement = new Pose(1, 0, 0);
        Pose measVar = new Pose(1, 1, 1);
        Pose initialCov = new Pose(1, 1, 1);

        // Scenario A: measure after a single update.
        FakeLocalizer odomA = new FakeLocalizer();
        TestableFusion a = new TestableFusion(odomA, initialCov, new Pose(1, 1, 1), measVar, 1000);
        a.setStartPose(new Pose(0, 0, 0));
        step(a, odomA, 1, new Pose(0, 0, 0));
        step(a, odomA, 2, new Pose(0, 0, 0));
        a.addMeasurement(measurement, 1, measVar);

        // Scenario B: measure after 50 stationary updates.
        FakeLocalizer odomB = new FakeLocalizer();
        TestableFusion b = new TestableFusion(odomB, initialCov, new Pose(1, 1, 1), measVar, 1000);
        b.setStartPose(new Pose(0, 0, 0));
        for (long t = 1; t <= 50; t++) step(b, odomB, t, new Pose(0, 0, 0));
        b.addMeasurement(measurement, 1, measVar);

        assertPoseEquals(a.getPose(), b.getPose(), 1e-9);
        // With P == R the gain is 0.5, so the estimate moves halfway to the measurement.
        assertEquals(0.5, a.getPose().getX(), 1e-9, "gain should reflect un-inflated covariance");
    }

    /**
     * Process noise scales with distance, so covering the same distance in one big step or many
     * small steps yields the same covariance, and thus the same correction from an equal measurement.
     */
    @Test
    void processNoise_isInvariantToStepCount() {
        Pose measurement = new Pose(0, 5, 0);
        Pose measVar = new Pose(2, 2, 2);
        Pose initialCov = new Pose(0.5, 0.5, 0.5);
        Pose processVar = new Pose(1, 1, 1);

        // One 10" step.
        FakeLocalizer odomA = new FakeLocalizer();
        TestableFusion a = new TestableFusion(odomA, initialCov, processVar, measVar, 1000);
        a.setStartPose(new Pose(0, 0, 0));
        step(a, odomA, 1, new Pose(10, 0, 0));
        step(a, odomA, 2, new Pose(10, 0, 0));
        a.addMeasurement(measurement, 1, measVar);

        // Ten 1" steps.
        FakeLocalizer odomB = new FakeLocalizer();
        TestableFusion b = new TestableFusion(odomB, initialCov, processVar, measVar, 1000);
        b.setStartPose(new Pose(0, 0, 0));
        long t = 1;
        for (int i = 1; i <= 10; i++) step(b, odomB, t++, new Pose(i, 0, 0));
        step(b, odomB, t, new Pose(10, 0, 0));
        b.addMeasurement(measurement, t - 1, measVar);

        assertPoseEquals(a.getPose(), b.getPose(), 1e-9);
    }

    // ---- #3: degenerate measurement variances -----------------------------------------------

    /** A measurement variance of 0 (and a second one) must not throw, and should still fuse. */
    @Test
    void zeroVarianceMeasurement_doesNotThrowAndFuses() {
        FakeLocalizer odom = new FakeLocalizer();
        TestableFusion fused = new TestableFusion(odom,
                new Pose(1, 1, 1), new Pose(1, 1, 1), new Pose(1, 1, 1), 1000);
        fused.setStartPose(new Pose(0, 0, 0));
        step(fused, odom, 1, new Pose(0, 0, 0));
        step(fused, odom, 2, new Pose(0, 0, 0));

        Pose zeroVar = new Pose(0, 0, 0);
        assertDoesNotThrow(() -> fused.addMeasurement(new Pose(3, 4, 0.5), 1, zeroVar));
        // Trusted measurement => estimate snaps essentially onto it.
        assertPoseEquals(new Pose(3, 4, 0.5), fused.getPose(), 1e-2);

        // A second zero-variance measurement on the (now collapsed) axes must also be safe.
        assertDoesNotThrow(() -> fused.addMeasurement(new Pose(3, 4, 0.5), 1, zeroVar));
    }

    /** NaN axes in a measurement are ignored; only the supplied axis is corrected. */
    @Test
    void partialMeasurement_onlyUpdatesSuppliedAxes() {
        FakeLocalizer odom = new FakeLocalizer();
        TestableFusion fused = new TestableFusion(odom,
                new Pose(1e6, 1e6, 1e6), new Pose(1, 1, 1), new Pose(EPS, EPS, EPS), 1000);
        fused.setStartPose(new Pose(0, 0, 0));
        step(fused, odom, 1, new Pose(5, 3, 0));
        step(fused, odom, 2, new Pose(5, 3, 0));

        // Only heading is measured (x and y are NaN).
        fused.addMeasurement(new Pose(Double.NaN, Double.NaN, 0.4), 1, new Pose(EPS, EPS, EPS));

        assertEquals(5, fused.getPose().getX(), 1e-9, "x must be untouched by a heading-only fix");
        assertEquals(3, fused.getPose().getY(), 1e-9, "y must be untouched by a heading-only fix");
        assertEquals(0.4, fused.getPose().getHeading(), 1e-3, "heading should track the measurement");
    }

    /** A measurement whose timestamp predates the buffer window is rejected, leaving the estimate intact. */
    @Test
    void staleMeasurement_outsideWindow_isIgnored() {
        FakeLocalizer odom = new FakeLocalizer();
        TestableFusion fused = new TestableFusion(odom,
                new Pose(1, 1, 1), new Pose(1, 1, 1), new Pose(1, 1, 1), 1000);
        fused.setStartPose(new Pose(0, 0, 0));
        step(fused, odom, 10, new Pose(2, 0, 0));

        Pose before = fused.getPose();
        // firstKey is 0 (start pose); a negative timestamp is before the window.
        fused.addMeasurement(new Pose(100, 100, 1), -5, new Pose(1, 1, 1));
        assertPoseEquals(before, fused.getPose(), 1e-12);
        assertTrue(before.roughlyEquals(new Pose(2, 0, 0), 1e-9));
    }

    // ---- core functionality -----------------------------------------------------------------

    /**
     * Latency compensation: a measurement timestamped in the past corrects the historical pose, and
     * the odometry motion accumulated since then is re-applied so the correction reaches the present.
     */
    @Test
    void delayedMeasurement_correctionPropagatesToCurrentPose() {
        FakeLocalizer odom = new FakeLocalizer();
        TestableFusion fused = new TestableFusion(odom,
                new Pose(1e6, 1e6, 1e6), new Pose(1, 1, 1), new Pose(EPS, EPS, EPS), 1000);
        fused.setStartPose(new Pose(0, 0, 0));

        // Drive straight along x; the present odometry pose is (15, 0, 0).
        step(fused, odom, 1, new Pose(5, 0, 0));
        step(fused, odom, 2, new Pose(10, 0, 0));
        step(fused, odom, 3, new Pose(15, 0, 0));

        // A delayed fix says "at t=1 you were really at (5, 2, 0)".
        fused.addMeasurement(new Pose(5, 2, 0), 1, new Pose(EPS, EPS, EPS));

        // The +2 y correction must carry through the 10" of x travel since t=1 => present ≈ (15, 2, 0).
        assertPoseEquals(new Pose(15, 2, 0), fused.getPose(), 1e-2);
    }

    /** A partially trusted measurement nudges the estimate part-way toward the measurement, not all the way. */
    @Test
    void measurement_gainBlendsEstimateAndMeasurement() {
        FakeLocalizer odom = new FakeLocalizer();
        // P == 3, R == 1  =>  gain = P/(P+R) = 0.75 on each axis (no motion, so P is unchanged).
        TestableFusion fused = new TestableFusion(odom,
                new Pose(3, 3, 3), new Pose(1, 1, 1), new Pose(1, 1, 1), 1000);
        fused.setStartPose(new Pose(0, 0, 0));
        step(fused, odom, 1, new Pose(0, 0, 0));
        step(fused, odom, 2, new Pose(0, 0, 0));

        fused.addMeasurement(new Pose(4, 8, 0), 1, new Pose(1, 1, 1));

        assertEquals(3.0, fused.getPose().getX(), 1e-9, "x = 0 + 0.75*(4-0)");
        assertEquals(6.0, fused.getPose().getY(), 1e-9, "y = 0 + 0.75*(8-0)");
    }

    /** {@code setPose} overrides the current estimate, and the override survives a subsequent update. */
    @Test
    void setPose_overridesEstimateAndContinues() {
        FakeLocalizer odom = new FakeLocalizer();
        TestableFusion fused = new TestableFusion(odom,
                new Pose(1, 1, 1), new Pose(1, 1, 1), new Pose(1, 1, 1), 1000);
        fused.setStartPose(new Pose(0, 0, 0));
        step(fused, odom, 1, new Pose(5, 0, 0));
        assertPoseEquals(new Pose(5, 0, 0), fused.getPose(), 1e-9);

        fused.setPose(new Pose(20, 10, 1));
        assertPoseEquals(new Pose(20, 10, 1), fused.getPose(), 1e-12);

        // No odometry motion since the reset => the estimate must stay put (no jump back to the old pose).
        step(fused, odom, 2, new Pose(20, 10, 1));
        assertPoseEquals(new Pose(20, 10, 1), fused.getPose(), 1e-9);
    }

    /** Velocity is sourced from the dead-reckoning localizer. */
    @Test
    void getVelocity_reflectsDeadReckoningVelocity() {
        FakeLocalizer odom = new FakeLocalizer();
        TestableFusion fused = new TestableFusion(odom,
                new Pose(1, 1, 1), new Pose(1, 1, 1), new Pose(1, 1, 1), 1000);
        fused.setStartPose(new Pose(0, 0, 0));

        odom.setVelocity(new Pose(1, 2, 0.5));
        step(fused, odom, 1, new Pose(0, 0, 0));

        assertPoseEquals(new Pose(1, 2, 0.5), fused.getVelocity(), 1e-9);
        Vector v = fused.getVelocityVector();
        assertEquals(1, v.getXComponent(), 1e-9);
        assertEquals(2, v.getYComponent(), 1e-9);
    }

    /** Pass-through getters delegate to the underlying dead-reckoning localizer. */
    @Test
    void delegatingGetters_passThroughToDeadReckoning() {
        FakeLocalizer odom = new FakeLocalizer();
        odom.imuHeading = 0.7;
        odom.angularVelocity = 1.5;
        TestableFusion fused = new TestableFusion(odom,
                new Pose(1, 1, 1), new Pose(1, 1, 1), new Pose(1, 1, 1), 1000);
        fused.setStartPose(new Pose(0, 0, 0));

        assertEquals(2, fused.getForwardMultiplier(), 0);
        assertEquals(3, fused.getLateralMultiplier(), 0);
        assertEquals(4, fused.getTurningMultiplier(), 0);
        assertEquals(0.7, fused.getIMUHeading(), 0);
        assertEquals(1.5, fused.getAngularVelocity(), 0);
    }

    /** {@code isNAN} reflects the fused position. */
    @Test
    void isNAN_reflectsCurrentPosition() {
        FakeLocalizer odom = new FakeLocalizer();
        TestableFusion fused = new TestableFusion(odom,
                new Pose(1, 1, 1), new Pose(1, 1, 1), new Pose(1, 1, 1), 1000);
        fused.setStartPose(new Pose(0, 0, 0));
        step(fused, odom, 1, new Pose(3, 4, 0));
        assertFalse(fused.isNAN());

        fused.setPose(new Pose(Double.NaN, 0, 0));
        assertTrue(fused.isNAN());
    }
}
