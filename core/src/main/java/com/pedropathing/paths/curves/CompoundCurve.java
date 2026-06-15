/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths.curves;

import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Piecewise;
import com.pedropathing.paths.tvalue.TValue;
import java.util.List;

public class CompoundCurve implements Curve {
    private final Piecewise<Curve> curves;

    public CompoundCurve(List<Curve> curves) {
        if (curves.isEmpty()) throw new IllegalArgumentException("Compound curve must have at least one curve.");
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
    public double remainingDistance(double t) {
        return curves.length() - distanceAt(t);
    }

    private double distanceAt(double t) { // TODO: verify
        double distanceTraveled = 0.0;
        double currentT = 0.0;

        for (Piecewise.Segment<Curve> segment : curves.segments()) {
            Curve curve = segment.value();
            double curveLength = curve.length();
            double nextT = currentT + curveLength;

            if (t <= currentT) {
                // t before segment
                break;
            } else if (t >= nextT) {
                // t after segment
                distanceTraveled += curveLength;
            } else {
                // t within segment
                double localT = curves.localT(t);
                distanceTraveled += curveLength - curve.remainingDistance(localT);
                break;
            }

            currentT = nextT;
        }

        return distanceTraveled;
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
