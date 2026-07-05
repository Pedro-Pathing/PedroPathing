/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths;

import static com.pedropathing.utils.Utils.*;

import com.pedropathing.config.Modifier;
import com.pedropathing.paths.curves.CompoundCurve;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.paths.tvalue.TValue;
import java.util.List;

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
        this(null, listOf(), listOf(paths));
    }

    @Override
    public double heading(@TValue double t) {
        if (interpolator != null) return interpolator.interpolate(curve, t);
        return paths.get(t).heading(paths.localT(t));
    }

    @Override
    public double derivative(@TValue double t) {
        if (interpolator != null) return interpolator.interpolate(curve, t);
        return paths.get(t).heading(paths.localT(t));
    }

    @Override
    public boolean hasHeading() {
        if (interpolator != null) return true;
        return paths.segments().stream().map(Piecewise.Segment::value).allMatch(Path::hasHeading);
    }

    public List<Piecewise.Segment<Path>> segments() {
        return paths.segments();
    }

    @Override
    protected List<PathSegment> getSegments(PathSegment.HeadingProvider parentHeading, List<Modifier> modifiers) {
        return paths.segments().stream()
                .flatMap(segment -> segment
                        .value()
                        .getSegments(convertChildHeading(parentHeading, segment), concat(modifiers, this.modifiers))
                        .stream())
                .collect(toUnmodifiableList());
    }

    private PathSegment.HeadingProvider convertChildHeading(
            PathSegment.HeadingProvider parentHeading, Piecewise.Segment<Path> segment) {
        double totalLength = curve.length();
        double childLength = segment.value().curve.length();
        double segStart = segment.startT();
        double ratio = childLength / totalLength;

        return new PathSegment.HeadingProvider() {
            @Override
            public double heading(double t) {
                return parentHeading.heading(segStart + t * ratio);
            }

            @Override
            public double derivative(double t) {
                return parentHeading.derivative(segStart + t * ratio);
            }
        };
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
