/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.follower;

import com.pedropathing.localization.MotionState;
import lombok.Value;

@Value
public class FollowState {
    MotionState motionState;
}
