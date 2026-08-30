/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths.interpolator;

import static com.pedropathing.utils.Angle.error;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.utils.Angle;

@FunctionalInterface
public interface Interpolator {
    Interpolator tangent = (curve, t) -> curve.tangent(t).theta();

    default Interpolator reverse() {
        Interpolator outer = this;
        return (curve, t) -> Angle.normalize(outer.interpolate(curve, t) + Math.PI);
    }

    static Interpolator constant(double heading) {
        double finalHeading = Angle.normalize(heading);
        return (Curve curve, double t) -> finalHeading;
    }

    static Interpolator constant(Pose pose) {
        return constant(pose.heading());
    }

    static Interpolator linear(double start, double end) {
        double finalStart = Angle.normalize(start);
        double finalEnd = Angle.normalize(end);
        double deltaHeading = error(finalStart, finalEnd);

        return (curve, t) -> Angle.normalize(finalStart + deltaHeading * curve.pathCompletion(t));
    }

    static Interpolator linear(Pose start, Pose end) {
        return linear(start.heading(), end.heading());
    }

    static Interpolator longLinear(double start, double end) {
        double finalStart = Angle.normalize(start);
        double finalEnd = Angle.normalize(end);

        double deltaHeading =
                -Angle.turnDirection(finalStart, finalEnd) * Angle.smallestDifference(finalStart, finalEnd);

        return (curve, t) -> Angle.normalize(finalStart + deltaHeading * curve.pathCompletion(t));
    }

    static Interpolator longLinear(Pose start, Pose end) {
        return longLinear(start.heading(), end.heading());
    }

    static Interpolator facingPoint(Vector2D point) {
        return (curve, t) -> point.minus(curve.get(t)).theta();
    }

    static Interpolator facingPoint(Pose pose) {
        return facingPoint(pose.toVector2D());
    }

    static PiecewiseInterpolator piecewise() {
        return new PiecewiseInterpolator();
    }

    double interpolate(Curve curve, double t);
}
