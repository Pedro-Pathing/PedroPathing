/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.localization;

import com.pedropathing.math.Pose;

/**
 * A localizer backed by an absolute pose sensor such as a camera.
 * <p>
 * Unlike a dead-reckoning localizer, a vision localizer only produces a new measurement when the
 * sensor successfully observes the field. {@link #pollMeasurement()} returns each measurement at
 * most once so that stale poses are not fused repeatedly.
 */
public interface VisionLocalizer extends Localizer {
    /**
     * Returns the next new measurement, or {@code null} if no new valid measurement is available.
     * <p>
     * Axes that the sensor cannot measure reliably should be set to {@link Double#NaN}. The pose is
     * in the same world frame as the rest of Pedro Pathing (inches and radians).
     */
    Pose pollMeasurement();

    /**
     * Returns the timestamp of the most recent measurement in the {@link System#nanoTime()} time
     * base. This is only meaningful when {@link #pollMeasurement()} returned a non-null pose.
     */
    long lastMeasurementTimestamp();

    /**
     * Returns the variance for the most recent measurement as {@code (x, y, heading)}, or
     * {@code null} to use the fusion localizer's default measurement variance.
     * <p>
     * These must be variances, not standard deviations.
     */
    default Pose lastMeasurementVariance() {
        return null;
    }

    /**
     * Updates the sensor using the current Pedro world-frame robot heading. Sensors that need an
     * orientation for their pose solve should use this heading instead of an independent IMU so
     * they stay synchronized with the fused estimate. Sensors whose pose solve expects a
     * different heading convention must convert it before using it. The default implementation
     * ignores the heading and calls {@link #update()}.
     */
    default void update(double robotHeading) {
        update();
    }
}
