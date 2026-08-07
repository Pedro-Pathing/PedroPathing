/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths;

import com.pedropathing.config.Modifier;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.curves.Curve;

import java.util.List;
import java.util.function.DoubleUnaryOperator;

public final class PathSegment {
    public final Curve curve;
    private final HeadingProvider heading;
    private final List<Modifier> modifiers;
    private final Pose endPose;

    PathSegment(Curve curve, HeadingProvider heading, List<Modifier> modifiers) {
        this.curve = curve;
        this.heading = heading;
        this.modifiers = modifiers;
        endPose = curve.get(1.0).toPose(heading.heading(1.0));
    }

    public double heading(double t) {
        TValue.check(t);
        return heading.heading(t);
    }

    public double headingDerivative(double t) {
        TValue.check(t);
        return heading.derivative(t);
    }

    public Pose get(double t) {
        TValue.check(t);
        return curve.get(t).toPose(heading(t));
    }

    List<Modifier> modifiers() {
        return modifiers;
    }

    public Pose endPose() {
        return endPose;
    }

    public interface HeadingProvider {
        double heading(double t);
        double derivative(double t);

        static HeadingProvider of(DoubleUnaryOperator heading, DoubleUnaryOperator derivative) {
            return new HeadingProvider() {
                @Override
                public double heading(double t) {
                    return heading.applyAsDouble(t);
                }

                @Override
                public double derivative(double t) {
                    return derivative.applyAsDouble(t);
                }
            };
        }
    }
}
