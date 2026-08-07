/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.drivetrain;

import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Vector2D;
import com.pedropathing.utils.Pair;
import com.pedropathing.utils.Utils;

public interface Drivetrain {
    void drive(DrivePowers powers, boolean manual);
    double maxScaling(DrivePowers current,
                             DrivePowers delta);
    void stop();
}
