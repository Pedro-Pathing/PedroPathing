/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.drivetrain;

public interface Drivetrain {
    void drive(DrivePowers powers, boolean manual);

    double[] computeWheelPowers(DrivePowers powers);

    double maxScaling(DrivePowers current, DrivePowers delta);

    void stop();
}
