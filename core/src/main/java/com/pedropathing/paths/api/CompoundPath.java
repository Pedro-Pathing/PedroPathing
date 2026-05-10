/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths.api;

import com.pedropathing.config.Modifier;
import com.pedropathing.paths.curves.CompoundCurve;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.paths.tvalue.TValue;
import java.util.Arrays;

public class CompoundPath extends Path {
    private final Interpolator interpolator;
    private final Piecewise<Path> paths;

    CompoundPath(Interpolator interpolator, Modifier[] modifiers, Path[] paths) {
        super(new CompoundCurve(Arrays.stream(paths).map(path -> path.curve).toArray(Curve[]::new)), modifiers);
        this.interpolator = interpolator;
        this.paths = new Piecewise<>(Path::length, paths);
    }

    @Override
    double getHeading(@TValue double t) {
        if (interpolator != null) return interpolator.interpolate(curve, t);
        else return paths.get(t).getHeading(paths.localT(t));
    }
}
