/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.drivetrain;

public interface Drivetrain {
    void drive(DrivePowers powers, boolean manual);

    void stop();
}
