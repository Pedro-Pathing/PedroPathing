/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.algorithm;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.PathTracker;

public interface Algorithm {
    DrivePowers calculatePath(PathTracker pathTracker, MotionState state, double deltaTime);

    DrivePowers calculateHold(Pose target, MotionState state, double deltaTime);
}
