/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.localization;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Velocity;

public interface Localizer {
    Pose getPose();

    void setPose(Pose pose);

    default void setX(double x) {
        setPose(getPose().withX(x));
    }

    default void setY(double y) {
        setPose(getPose().withY(y));
    }

    default void setHeading(double heading) {
        setPose(getPose().withHeading(heading));
    }

    Twist getTwist();

    Velocity getVelocity();

    void update();
}
