/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths.curves;

import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Piecewise;
import com.pedropathing.paths.tvalue.TValue;

public class CompoundCurve implements Curve {
    private final Piecewise<Curve> curves;

    public CompoundCurve(Curve... curves) {
        if (curves.length == 0) throw new IllegalArgumentException("Compound curve must have at least one curve.");
        this.curves = new Piecewise<>(Curve::length, curves);
    }

    @Override
    public Vector2D get(@TValue double t) {
        return curves.get(t).get(curves.localT(t));
    }

    @Override
    public double closestT(Vector2D position) {
        double bestT = 0.0;
        double bestDistance = Double.POSITIVE_INFINITY;

        for (Piecewise.Segment<Curve> segment : curves.segments()) {
            Curve curve = segment.value();
            double localT = curve.closestT(position);

            Vector2D point = curve.get(localT);
            double distance = point.distance(position);

            if (distance < bestDistance) {
                bestDistance = distance;
                bestT = curves.globalT(segment, localT);
            }
        }

        return bestT;
    }

    @Override
    public double length() {
        return curves.length();
    }

    @Override
    public Vector2D tangent(double t) {
        return curves.get(t).tangent(curves.localT(t));
    }

    @Override
    public double curvature(double t) {
        return curves.get(t).curvature(curves.localT(t));
    }
}
