/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.drivetrain;

import com.pedropathing.algorithm.Algorithm;

public interface Drivetrain {
    void drive(DrivePowers powers, Algorithm algorithm);
    // if (algorithm instanceof TeleOpAlgorithm && constants.usebrake)
    void stop();
}
