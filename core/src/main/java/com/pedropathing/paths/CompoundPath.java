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

import java.util.List;

import static com.pedropathing.utils.Utils.listOf;
import static com.pedropathing.utils.Utils.toUnmodifiableList;

public class CompoundPath extends Path {
    private final Interpolator interpolator;
    private final Piecewise<Path> paths;

    private CompoundPath(Curve curve, Interpolator interpolator, List<Modifier> modifiers, Piecewise<Path> paths) {
        super(curve, modifiers);
        this.interpolator = interpolator;
        this.paths = paths;
    }

    public CompoundPath(Interpolator interpolator, List<Modifier> modifiers, List<Path> paths) {
        super(new CompoundCurve(paths.stream().map(path -> path.curve).collect(toUnmodifiableList())), modifiers);
        this.interpolator = interpolator;
        this.paths = new Piecewise<>(path -> path.curve.length(), paths);
    }

    public CompoundPath(Path... paths) {
        this(null, null, listOf(paths));
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
                .collect(toUnmodifiableList());
    }

    @Override
    protected Path withHeading(Interpolator interpolator) {
        return new CompoundPath(curve, interpolator, modifiers, paths);
    }

    @Override
    protected Path withModifiers(List<Modifier> modifiers) {
        return new CompoundPath(curve, interpolator, modifiers, paths);
    }
}
