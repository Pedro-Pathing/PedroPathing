/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.localization;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Velocity;

public interface Localizer {
    void setPose(Pose pose);

    default void setX(double x) {
        setPose(pose().withX(x));
    }

    default void setY(double y) {
        setPose(pose().withY(y));
    }

    default void setHeading(double heading) {
        setPose(pose().withHeading(heading));
    }

    default Pose pose() {
        return state().pose();
    }

    default Twist twist() {
        return state().twist();
    }

    default Velocity velocity() {
        return state().velocity();
    }

    MotionState state();

    void update();

    void reset();
}
