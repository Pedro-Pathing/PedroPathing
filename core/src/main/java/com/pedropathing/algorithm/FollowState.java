/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.algorithm;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.compiled.PathProgress;
import lombok.Value;

@Value
public class FollowState {
    Pose pose;
    Velocity velocity;
    Twist twist;
    PathProgress pathProgress;
}
