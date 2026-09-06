/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.drivetrain;

public interface Drivetrain {
    void drive(DrivePowers powers, boolean manual);

    double maxScaling(DrivePowers current, DrivePowers delta);

    void stop();

    String debugString();

    default double interpolateAcceleration(double xRadius, double yRadius, double theta) {
        double cos = Math.abs(Math.cos(theta));
        double cos3 = cos * cos * cos;
        double sin = Math.abs(Math.sin(theta));
        double sin3 = sin * sin * sin;
        return 1.0 / (Math.abs(cos3) / xRadius + Math.abs(sin3) / yRadius);
    }

    double interpolateVelocity(double xRadius, double yRadius, double theta);
}
