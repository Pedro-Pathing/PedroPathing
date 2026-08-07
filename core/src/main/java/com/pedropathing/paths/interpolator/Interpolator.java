/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths.interpolator;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.utils.Angle;

@FunctionalInterface
public interface Interpolator {
    Interpolator tangent = new Interpolator() {
        @Override
        public double interpolate(Curve curve, double t) {
            return curve.tangent(t).theta();
        }

        @Override
        public double differentiate(Curve curve, double t) {
            return curve.curvature(t) * curve.derivative(t).magnitude();
        }
    };

    default Interpolator reverse() {
        Interpolator outer = this;
        return new Interpolator() {
            @Override
            public double interpolate(Curve curve, double t) {
                return Angle.normalize(outer.interpolate(curve, t) + Math.PI);
            }

            @Override
            public double differentiate(Curve curve, double t) {
                return outer.differentiate(curve, t);
            }
        };
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
        double deltaHeading =
                Angle.turnDirection(finalStart, finalEnd) * Angle.smallestDifference(finalStart, finalEnd);

        return new Interpolator() {
            @Override
            public double interpolate(Curve curve, double t) {
                return Angle.normalize(finalStart + deltaHeading * t);
            }

            @Override
            public double differentiate(Curve curve, double t) {
                return deltaHeading;
            }
        };
    }

    static Interpolator linear(Pose start, Pose end) {
        return linear(start.heading(), end.heading());
    }

    static Interpolator facingPoint(Vector2D point) {
        return new Interpolator() {
            @Override
            public double interpolate(Curve curve, double t) {
                return point.minus(curve.get(t)).theta();
            }

            @Override
            public double differentiate(Curve curve, double t) {
                Vector2D r = point.minus(curve.get(t));
                if (r.isZero()) return 0;
                return -r.det(curve.derivative(t)) / r.dot(r);
            }
        };
    }

    static Interpolator facingPoint(Pose pose) {
        return facingPoint(pose.toVector2D());
    }

    static PiecewiseInterpolator piecewise() {
        return new PiecewiseInterpolator();
    }

    double interpolate(Curve curve, double t);

    default double differentiate(Curve curve, double t) {
        // TODO: It's only necessary to have this method in an Algorithm that uses it
        return 0;
    }
}
