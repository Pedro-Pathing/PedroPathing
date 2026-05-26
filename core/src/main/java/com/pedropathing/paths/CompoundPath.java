/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths;

import com.pedropathing.config.Modifier;
import com.pedropathing.paths.curves.CompoundCurve;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.paths.tvalue.TValue;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class CompoundPath extends Path {
    private final Interpolator interpolator;
    private final Piecewise<Path> paths;

    public CompoundPath(Interpolator interpolator, Modifier[] modifiers, Path[] paths) {
        super(new CompoundCurve(Arrays.stream(paths).map(path -> path.curve).toArray(Curve[]::new)), modifiers);
        this.interpolator = interpolator;
        this.paths = new Piecewise<>(Path::length, paths);
    }

    @Override
    public double heading(@TValue double t) {
        if (interpolator != null) return interpolator.interpolate(curve, t);
        else return paths.get(t).heading(paths.localT(t));
    }

    public List<Piecewise.Segment<Path>> segments() {
        return paths.segments();
    }

    @Override
    public List<AtomicPath> getPaths() {
        return paths.segments().stream()
                .map(Piecewise.Segment::value)
                .flatMap(path -> path.getPaths().stream())
                .collect(Collectors.toList());
    }
}
