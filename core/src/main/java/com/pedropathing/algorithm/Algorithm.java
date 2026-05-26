/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.algorithm;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.FollowState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;

public interface Algorithm {
    DrivePowers calculate(FollowState state);
    DrivePowers hold(Pose target, FollowState state);
}
