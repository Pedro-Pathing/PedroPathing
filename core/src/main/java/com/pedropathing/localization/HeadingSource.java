/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.localization;

/**
 * Supplies a single heading sample in radians.
 *
 * <p>Implementations may read an IMU, a simulator, a replay, or a cycle-level cache.
 */
@FunctionalInterface
public interface HeadingSource {
    /**
     * Current heading in radians.
     *
     * @return heading in radians
     */
    double headingRadians();
}
