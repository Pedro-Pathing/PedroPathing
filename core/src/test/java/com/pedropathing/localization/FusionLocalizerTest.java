/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.localization;

import static com.google.common.truth.Truth.assertThat;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import org.junit.jupiter.api.Test;

class FusionLocalizerTest {
    private static final double EPSILON = 1e-9;

    @Test
    void updateUsesFusedHeadingForVisionAndAppliesMeasurement() {
        FakeDeadReckoning deadReckoning = new FakeDeadReckoning();
        FakeVision vision = new FakeVision();
        FusionLocalizer fusion =
                new FusionLocalizer(deadReckoning, vision, new Pose(1, 1, 1), new Pose(1, 1, 1), new Pose(1, 1, 1), 20);

        fusion.setPose(Pose.zero());
        vision.pendingMeasurement = new Pose(10, 0, Double.NaN);
        vision.timestamp = System.nanoTime();

        fusion.update();

        assertThat(vision.updatedHeading).isEqualTo(0.0);
        assertThat(vision.pendingMeasurement).isNull();
        assertThat(fusion.pose().x()).isGreaterThan(0.0);
        assertThat(fusion.pose().x()).isLessThan(10.0);
    }

    @Test
    void poseAndResetAreForwardedToVision() {
        FakeVision vision = new FakeVision();
        FusionLocalizer fusion = new FusionLocalizer(
                new FakeDeadReckoning(), vision, new Pose(1, 1, 1), new Pose(1, 1, 1), new Pose(1, 1, 1), 20);

        Pose pose = new Pose(12, 24, 0.5);
        fusion.setPose(pose);

        assertThat(vision.lastSetPose).isSameInstanceAs(pose);

        fusion.reset();

        assertThat(vision.wasReset).isTrue();
    }

    @Test
    void measurementAtOlderTimestampCorrectsPropagatedHistory() throws Exception {
        FakeDeadReckoning deadReckoning = new FakeDeadReckoning();
        deadReckoning.stepX = 10.0;
        FusionLocalizer fusion =
                new FusionLocalizer(deadReckoning, new Pose(1, 1, 1), new Pose(1, 1, 1), new Pose(1, 1, 1), 20);

        fusion.setPose(Pose.zero());

        long[] times = new long[4];
        for (int i = 0; i < 4; i++) {
            Thread.sleep(5);
            times[i] = System.nanoTime();
            fusion.update();
        }
        assertThat(fusion.pose().x()).isWithin(EPSILON).of(40.0);

        // Land the measurement between the second and third history entries, where the raw
        // estimate is between 20 and 30, and have the vision sensor report x = 100 there.
        long midTimestamp = times[1] + (times[2] - times[1]) / 2;
        fusion.addMeasurement(new Pose(100, 0, Double.NaN), midTimestamp);

        // With P = R = 1 the correction is fused halfway toward the measurement at that time,
        // then re-propagated through the remaining raw motion. The result must land well past
        // the uncorrected 40 but short of the measurement itself.
        assertThat(fusion.pose().x()).isAtLeast(74.0);
        assertThat(fusion.pose().x()).isLessThan(81.0);
        assertThat(fusion.pose().y()).isWithin(EPSILON).of(0.0);
        assertThat(fusion.pose().heading()).isWithin(EPSILON).of(0.0);
    }

    @Test
    void measurementOutsideHistoryWindowIsRejected() {
        FakeDeadReckoning deadReckoning = new FakeDeadReckoning();
        deadReckoning.stepX = 10.0;
        FusionLocalizer fusion =
                new FusionLocalizer(deadReckoning, new Pose(1, 1, 1), new Pose(1, 1, 1), new Pose(1, 1, 1), 20);

        fusion.setPose(Pose.zero());
        long before = System.nanoTime();
        fusion.update();

        // Ten seconds before the initial history entry.
        fusion.addMeasurement(new Pose(1000, 0, Double.NaN), before - 10_000_000_000L);
        // And ten seconds after the newest entry.
        fusion.addMeasurement(new Pose(1000, 0, Double.NaN), System.nanoTime() + 10_000_000_000L);

        assertThat(fusion.pose().x()).isWithin(EPSILON).of(10.0);
        assertThat(fusion.pose().y()).isWithin(EPSILON).of(0.0);
    }

    @Test
    void futureVisionTimestampIsClampedToTheCurrentState() {
        FakeVision vision = new FakeVision();
        FusionLocalizer fusion = new FusionLocalizer(
                new FakeDeadReckoning(), vision, new Pose(1, 1, 1), new Pose(1, 1, 1), new Pose(1, 1, 1), 20);

        fusion.setPose(Pose.zero());

        // A timestamp newer than the history entry update() just added is clamped back to it
        // instead of being rejected, so the measurement fuses at the freshest estimate.
        vision.pendingMeasurement = new Pose(10, 0, Double.NaN);
        vision.timestamp = System.nanoTime() + 1_000_000_000L;
        fusion.update();

        // P = R = 1 fuses halfway from 0 toward 10.
        assertThat(fusion.pose().x()).isWithin(EPSILON).of(5.0);
    }

    @Test
    void nanMeasurementAxesAreMasked() {
        FakeVision vision = new FakeVision();
        FusionLocalizer fusion = new FusionLocalizer(
                new FakeDeadReckoning(), vision, new Pose(1, 1, 1), new Pose(1, 1, 1), new Pose(1, 1, 1), 20);

        fusion.setPose(new Pose(5, 5, 0));
        long before = System.nanoTime();
        fusion.update();

        // The sensor only measures x; NaN in y and heading must not corrupt those axes.
        vision.pendingMeasurement = new Pose(100, Double.NaN, Double.NaN);
        vision.timestamp = before;
        fusion.update();

        assertThat(fusion.pose().x()).isWithin(EPSILON).of(52.5);
        assertThat(fusion.pose().y()).isWithin(EPSILON).of(5.0);
        assertThat(fusion.pose().heading()).isWithin(EPSILON).of(0.0);
        assertThat(vision.pendingMeasurement).isNull();
    }

    @Test
    void visionMeasurementVarianceIsForwardedToFusion() {
        FakeVision vision = new FakeVision();
        FusionLocalizer fusion = new FusionLocalizer(
                new FakeDeadReckoning(), vision, new Pose(1, 1, 1), new Pose(1, 1, 1), new Pose(1, 1, 1), 20);

        fusion.setPose(Pose.zero());

        vision.pendingMeasurement = new Pose(100, 0, Double.NaN);
        vision.timestamp = System.nanoTime();
        vision.returnVariance = new Pose(1e6, 1e6, 1e6);
        fusion.update();

        // A huge measurement variance means almost no trust in the measurement.
        assertThat(fusion.pose().x()).isGreaterThan(0.0);
        assertThat(fusion.pose().x()).isLessThan(0.01);
    }

    @Test
    void customMeasurementVarianceScalesCorrectionStrength() {
        FusionLocalizer weak = new FusionLocalizer(
                new FakeDeadReckoning(), new Pose(1, 1, 1), new Pose(1, 1, 1), new Pose(1, 1, 1), 20);
        weak.setPose(Pose.zero());
        long weakTimestamp = updateOnce(weak);
        weak.addMeasurement(new Pose(100, 0, Double.NaN), weakTimestamp, new Pose(1e6, 1e6, 1e6));
        assertThat(weak.pose().x()).isGreaterThan(0.0);
        assertThat(weak.pose().x()).isLessThan(0.01);

        FusionLocalizer strong = new FusionLocalizer(
                new FakeDeadReckoning(), new Pose(1, 1, 1), new Pose(1, 1, 1), new Pose(1, 1, 1), 20);
        strong.setPose(Pose.zero());
        long strongTimestamp = updateOnce(strong);
        strong.addMeasurement(new Pose(100, 0, Double.NaN), strongTimestamp, new Pose(1e-4, 1e-4, 1e-4));
        assertThat(strong.pose().x()).isGreaterThan(99.0);
        assertThat(strong.pose().x()).isLessThan(100.0);
    }

    @Test
    void setStartPoseResetsHistoryAndForwardsToVision() {
        FakeDeadReckoning deadReckoning = new FakeDeadReckoning();
        deadReckoning.stepX = 10.0;
        FakeVision vision = new FakeVision();
        FusionLocalizer fusion =
                new FusionLocalizer(deadReckoning, vision, new Pose(1, 1, 1), new Pose(1, 1, 1), new Pose(1, 1, 1), 20);

        Pose start = new Pose(3, 4, 0.5);
        fusion.setStartPose(start);

        assertThat(vision.lastSetPose).isSameInstanceAs(start);
        assertThat(fusion.pose().x()).isWithin(EPSILON).of(3.0);
        assertThat(fusion.pose().y()).isWithin(EPSILON).of(4.0);
        assertThat(fusion.pose().heading()).isWithin(EPSILON).of(0.5);

        fusion.update();

        // The filter resumes from the start pose and follows the raw odometry deltas.
        assertThat(fusion.pose().x()).isWithin(EPSILON).of(13.0);
        assertThat(fusion.pose().y()).isWithin(EPSILON).of(4.0);
        assertThat(fusion.pose().heading()).isWithin(EPSILON).of(0.5);
    }

    @Test
    void correctPoseShiftsEstimateAndFollowsDeadReckoning() {
        FakeDeadReckoning deadReckoning = new FakeDeadReckoning();
        deadReckoning.stepX = 10.0;
        FakeVision vision = new FakeVision();
        FusionLocalizer fusion =
                new FusionLocalizer(deadReckoning, vision, new Pose(1, 1, 1), new Pose(1, 1, 1), new Pose(1, 1, 1), 20);

        fusion.setPose(new Pose(5, 5, 0));
        Pose corrected = new Pose(7, 7, 0.25);
        fusion.correctPose(corrected);

        assertThat(fusion.pose().x()).isWithin(EPSILON).of(7.0);
        assertThat(fusion.pose().y()).isWithin(EPSILON).of(7.0);
        assertThat(fusion.pose().heading()).isWithin(EPSILON).of(0.25);
        assertThat(vision.lastSetPose).isSameInstanceAs(corrected);

        fusion.update();

        // The next raw odometry delta continues from the corrected estimate.
        assertThat(fusion.pose().x()).isWithin(EPSILON).of(17.0);
        assertThat(fusion.pose().y()).isWithin(EPSILON).of(7.0);
        assertThat(fusion.pose().heading()).isWithin(EPSILON).of(0.25);
    }

    @Test
    void historyIsEvictedToBufferSize() throws Exception {
        FakeDeadReckoning deadReckoning = new FakeDeadReckoning();
        deadReckoning.stepX = 10.0;
        FusionLocalizer fusion =
                new FusionLocalizer(deadReckoning, new Pose(1, 1, 1), new Pose(1, 1, 1), new Pose(1, 1, 1), 2);

        fusion.setPose(Pose.zero());

        long[] times = new long[5];
        for (int i = 0; i < 5; i++) {
            Thread.sleep(2);
            times[i] = System.nanoTime();
            fusion.update();
        }
        assertThat(fusion.pose().x()).isWithin(EPSILON).of(50.0);

        // A measurement from between the first two updates is older than the surviving window
        // (only the last two entries remain) and must be rejected outright.
        long evictedTimestamp = times[0] + (times[1] - times[0]) / 2;
        fusion.addMeasurement(new Pose(1000, 0, Double.NaN), evictedTimestamp);
        assertThat(fusion.pose().x()).isWithin(EPSILON).of(50.0);

        // A measurement inside the surviving window is still fused and re-propagated.
        long recentTimestamp = times[3] + (times[4] - times[3]) / 2;
        fusion.addMeasurement(new Pose(100, 0, Double.NaN), recentTimestamp);
        assertThat(fusion.pose().x()).isAtLeast(74.0);
        assertThat(fusion.pose().x()).isLessThan(86.0);
    }

    @Test
    void visionLocalizerDefaultsDelegateUpdateAndReturnNullVariance() {
        DefaultVision vision = new DefaultVision();

        vision.update(1.23);

        assertThat(vision.updated).isTrue();
        assertThat(vision.lastMeasurementVariance()).isNull();
    }

    /** Runs one update and returns a timestamp taken just before it, which is inside the history window. */
    private static long updateOnce(FusionLocalizer fusion) {
        long before = System.nanoTime();
        fusion.update();
        return before;
    }

    private static class FakeDeadReckoning implements Localizer {
        private Pose pose = Pose.zero();
        private Velocity velocity = Velocity.zero();
        /** World-frame x translation applied on every update, so history entries diverge over time. */
        private double stepX;

        @Override
        public void setPose(Pose pose) {
            this.pose = pose;
        }

        @Override
        public MotionState state() {
            return MotionState.ofVelocity(pose, velocity);
        }

        @Override
        public void update() {
            if (stepX != 0) pose = pose.withX(pose.x() + stepX);
        }

        @Override
        public void reset() {
            pose = Pose.zero();
        }
    }

    private static class FakeVision implements VisionLocalizer {
        private Pose pendingMeasurement;
        private long timestamp;
        private Pose returnVariance;
        private double updatedHeading = Double.NaN;
        private Pose lastSetPose;
        private boolean wasReset;

        @Override
        public Pose pollMeasurement() {
            Pose measurement = pendingMeasurement;
            pendingMeasurement = null;
            return measurement;
        }

        @Override
        public long lastMeasurementTimestamp() {
            return timestamp;
        }

        @Override
        public Pose lastMeasurementVariance() {
            return returnVariance;
        }

        @Override
        public void update(double robotHeading) {
            updatedHeading = robotHeading;
        }

        @Override
        public void setPose(Pose pose) {
            lastSetPose = pose;
        }

        @Override
        public MotionState state() {
            return MotionState.zero();
        }

        @Override
        public void update() {}

        @Override
        public void reset() {
            wasReset = true;
        }
    }

    private static class DefaultVision implements VisionLocalizer {
        private boolean updated;

        @Override
        public void update() {
            updated = true;
        }

        @Override
        public Pose pollMeasurement() {
            return null;
        }

        @Override
        public long lastMeasurementTimestamp() {
            return 0;
        }

        @Override
        public void setPose(Pose pose) {}

        @Override
        public MotionState state() {
            return MotionState.zero();
        }

        @Override
        public void reset() {}
    }
}
