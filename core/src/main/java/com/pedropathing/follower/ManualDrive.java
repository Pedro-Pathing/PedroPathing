/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.follower;

import static com.pedropathing.utils.Angle.normalizeSigned;

import com.pedropathing.controllers.Controller;
import com.pedropathing.controllers.PIDController;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Vector2D;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Utils;

public class ManualDrive {
    /** Takes in robotCentric drive powers and uses the currentHeading to rotate them to fieldCentric drive powers. */
    public static DrivePowers fieldCentric(DrivePowers powers, double currentHeading) {
        return fieldCentric(powers, currentHeading, 0.0);
    }

    /** Takes in robotCentric drive powers and uses the currentHeading to rotate them to fieldCentric drive powers with an offset heading. */
    public static DrivePowers fieldCentric(DrivePowers powers, double currentHeading, double offsetHeading) {
        Vector2D fieldRelative =
                Vector2D.cartesian(powers.forward(), powers.strafe()).rotate(-(currentHeading + offsetHeading));
        return new DrivePowers(fieldRelative.x(), fieldRelative.y(), powers.turn());
    }

    /** Takes in robotCentric drive powers and uses the currentHeading to rotate them to fieldCentric drive powers. */
    public static DrivePowers fieldCentric(double forward, double lateral, double turn, double currentHeading) {
        return fieldCentric(new DrivePowers(forward, lateral, turn), currentHeading);
    }

    /** Takes in robotCentric drive powers and uses the currentHeading to rotate them to fieldCentric drive powers with an offset heading. */
    public static DrivePowers fieldCentric(
            double forward, double lateral, double turn, double currentHeading, double offsetHeading) {
        return fieldCentric(new DrivePowers(forward, lateral, turn), currentHeading, offsetHeading);
    }

    /** Implements a heading lock by using the headingPID to calculate a turn power based on the targetHeading and current heading. */
    public static DrivePowers headingLock(
            Follower follower, PIDController headingPID, DrivePowers powers, double targetHeading) {
        double headingError = normalizeSigned(targetHeading - follower.pose().heading());
        double power = headingPID.calculate(targetHeading, headingError, follower.twist().omega);
        return new DrivePowers(powers.forward(), powers.strafe(), power);
    }

    /** Implements a heading lock by using the headingController to calculate a turn power based on the targetHeading and current heading. */
    public static DrivePowers headingLock(
            Follower follower, Controller headingController, DrivePowers powers, double targetHeading) {
        double headingError = normalizeSigned(targetHeading - follower.pose().heading());
        double power = headingController.calculate(targetHeading, headingError, follower.twist().omega);
        return new DrivePowers(powers.forward(), powers.strafe(), power);
    }

    /** Implements a heading lock using a predictive model of heading to calculate a turn power based on the targetHeading and current heading. */
    public static DrivePowers headingLock(
            Follower follower, Controller headingFeedback, DrivePowers powers, double targetHeading,
            double headingLinear, double headingQuadratic, double strength) {
        double angularVel = follower.velocity().omega;
        double brakeDist = headingLinear * angularVel +
                headingQuadratic * angularVel * angularVel * Math.signum(angularVel);
        double headingError = Angle.normalizeSigned(targetHeading - follower.pose().heading());
        double error = headingError - brakeDist;
        double power = Utils.clamp(headingFeedback.calculate(0, error), -0.3, 1.0) * strength;
        return new DrivePowers(powers.forward(), powers.strafe(), power);
    }

    public static DrivePowers headingLock(
            Follower follower, Controller headingFeedback, DrivePowers powers, double targetHeading,
            double headingLinear, double headingQuadratic) {
        return headingLock(follower, headingFeedback, powers, targetHeading, headingLinear, headingQuadratic, 0.5);
    }
}
