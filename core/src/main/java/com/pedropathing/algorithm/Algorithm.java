/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.algorithm;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.PathTracker;

public interface Algorithm {
    DrivePowers calculatePath(Drivetrain drivetrain, PathTracker pathTracker, MotionState state, double deltaTime);

    DrivePowers calculateHold(
            Drivetrain drivetrain, Pose target, MotionState state, boolean useScaling, double deltaTime);

    double completion();

    Pose closestPose();

    Vector2D closestTangent();

    Vector2D closestNormal();

    double curvature();

    double remainingDistance();

    default double parametricCompletion() {
        return completion();
    }

    boolean atParametricEnd();

    void reset();

    boolean isBusy();

    String debugString();
}
