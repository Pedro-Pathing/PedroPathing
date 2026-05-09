/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths.curves;

import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.tvalue.TValue;

import java.util.*;

public class CompoundCurve implements Curve {
    private final List<Curve> curves;
    private final double totalLength;
    private final NavigableMap<Double, Curve> curveMap = new TreeMap<>();

    public CompoundCurve(Curve... curves) {
        if (curves.length == 0) throw new IllegalArgumentException("Compound curve must have at least one curve.");
        this.curves = Arrays.asList(curves);
        totalLength = this.curves.stream().mapToDouble(Curve::length).sum();

        double currentT = 0.0;
        for (Curve curve : curves) {
            curveMap.put(currentT, curve);
            currentT += curve.length() / totalLength;
        }
    }

    private Curve getCurve(@TValue double t) {
        return curveMap.floorEntry(t).getValue();
    }

    private double getLocalT(@TValue double t) {
        return (t - curveMap.floorKey(t)) / curveMap.floorEntry(t).getValue().length() * totalLength;
    }

    @Override
    public Vector2D get(@TValue double t) {
        return getCurve(t).get(getLocalT(t));
    }

    @Override
    public double closestT(Vector2D position) {
        double bestT = 0.0;
        double bestDistance = Double.POSITIVE_INFINITY;

        for (Map.Entry<Double, Curve> entry : curveMap.entrySet()) {
            double startT = entry.getKey();
            Curve curve = entry.getValue();

            double localT = curve.closestT(position);

            Vector2D point = curve.get(localT);
            double distance = point.distance(position);

            if (distance < bestDistance) {
                bestDistance = distance;
                bestT = startT + localT * curve.length() / totalLength;
            }
        }

        return bestT;
    }

    @Override
    public double length() {
        return totalLength;
    }

    @Override
    public Vector2D tangent(double t) {
        return getCurve(t).tangent(getLocalT(t));
    }

    @Override
    public double curvature(double t) {
        return getCurve(t).curvature(getLocalT(t));
    }
}
