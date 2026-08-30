/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths.interpolator;

import com.pedropathing.paths.TValue;
import com.pedropathing.paths.curves.Curve;

import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

public class PiecewiseInterpolator implements Interpolator {
    private final NavigableMap<Double, Interpolator> interpolators = new TreeMap<>();
    private double greatestT = 0.0;

    PiecewiseInterpolator() {}

    public PiecewiseInterpolator until(double t, Interpolator interpolator) {
        if (t <= greatestT)
            throw new IllegalArgumentException(
                    "t was " + t + " but  must be greater than " + greatestT + ", the greatest t already defined.");
        if (t > 1.0) throw new IllegalArgumentException("t must be less than or equal to 1.0.");
        greatestT = t;
        interpolators.put(t, interpolator);
        return this;
    }

    @Override
    public double interpolate(Curve curve, double t) {
        if (greatestT < 1.0)
            throw new IllegalStateException("piecewise interpolation must be fully defined before interpolating.");
        TValue.check(t);

        double completion = curve.pathCompletion(t);

        Map.Entry<Double, Interpolator> entry = interpolators.ceilingEntry(completion);
        Map.Entry<Double, Interpolator> previous = interpolators.lowerEntry(entry.getKey());

        double initialT = previous == null ? 0.0 : curve.parameter(previous.getKey());
        double finalT = curve.parameter(entry.getKey());

        double normalizedT = (t - initialT) / (finalT - initialT);
        return entry.getValue().interpolate(curve, normalizedT);
    }
}
