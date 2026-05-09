/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths;

import com.pedropathing.paths.curves.CompoundCurve;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.paths.tvalue.TValue;

import java.util.Arrays;
import java.util.NavigableMap;
import java.util.TreeMap;

public class CompoundPath extends Path {
    private final Interpolator interpolator;
    private final NavigableMap<Double, Path> pathMap = new TreeMap<>();

    CompoundPath(Interpolator interpolator, Path... paths) {
        super(new CompoundCurve(
                Arrays.stream(paths).map(path -> path.curve).toArray(Curve[]::new)
        ));
        this.interpolator = interpolator;
        double currentT = 0.0;
        for (Path path : paths) {
            pathMap.put(currentT, path);
            currentT += curve.length() / this.length();
        }
    }

    private Path getPath(@TValue double t) {
        return pathMap.floorEntry(t).getValue();
    }

    private double getLocalT(@TValue double t) {
        return (t - pathMap.floorKey(t)) / pathMap.floorEntry(t).getValue().length() * this.length();
    }

    @Override
    double getHeading(@TValue double t) {
        if (interpolator != null) return interpolator.interpolate(curve, t);
        else return getPath(t).getHeading(getLocalT(t));
    }
}