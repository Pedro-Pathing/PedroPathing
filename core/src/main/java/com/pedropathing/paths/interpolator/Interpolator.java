/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths.interpolator;

import com.pedropathing.math.Angles;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.tvalue.TValue;

@FunctionalInterface
public interface Interpolator {
    Interpolator tangent = (Curve curve, @TValue double t) -> curve.tangent(t).theta();

    static Interpolator constant(double heading) {
        double finalHeading = Angles.normalize(heading);
        return (Curve curve, @TValue double t) -> finalHeading;
    }

    static Interpolator linear(double start, double end) {
        double finalStart = Angles.normalize(start);
        double finalEnd = Angles.normalize(end);
        return (Curve curve, @TValue double t) -> {
            double deltaHeading =
                    Angles.turnDirection(finalStart, finalEnd) * Angles.smallestDifference(finalStart, finalEnd);
            return Angles.normalize(finalStart + deltaHeading * t);
        };
    }

    static Interpolator facingPose(Pose pose) {
        return (Curve curve, @TValue double t) ->
                pose.toVector2D().minus(curve.get(t)).theta();
    }

    static PiecewiseInterpolator piecewise() {
        return new PiecewiseInterpolator();
    }

    double interpolate(Curve curve, @TValue double t);
}
