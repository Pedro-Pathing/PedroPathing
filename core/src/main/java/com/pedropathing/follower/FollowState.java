/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.follower;

import com.pedropathing.localization.MotionState;
import com.pedropathing.paths.PathTracker;
import lombok.Value;

@Value
public class FollowState {
    MotionState motionState;
    PathTracker pathTracker;
    double deltaTime;
}
