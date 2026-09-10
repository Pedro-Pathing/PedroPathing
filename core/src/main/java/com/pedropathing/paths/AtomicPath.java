/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths;

import static com.pedropathing.utils.Utils.concat;
import static com.pedropathing.utils.Utils.listOf;

import com.pedropathing.config.Modifier;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.interpolator.Interpolator;
import java.util.List;

public class AtomicPath extends Path {
    private final Interpolator interpolator;

    public AtomicPath(Curve curve, Interpolator interpolator, List<Modifier> modifiers) {
        super(curve, modifiers);
        this.interpolator = interpolator;
    }

    public AtomicPath(Curve curve) {
        this(curve, null, listOf());
    }

    @Override
    protected boolean hasHeading() {
        return interpolator != null;
    }

    @Override
    public double heading(double t) {
        TValue.check(t);
        if (interpolator == null) throw new UnsupportedOperationException("No heading interpolator set.");
        return interpolator.interpolate(curve, t);
    }

    @Override
    protected List<PathSegment> getSegments(PathSegment.HeadingProvider parentHeading, List<Modifier> modifiers) {
        return listOf(new PathSegment(this.curve, parentHeading, concat(modifiers, this.modifiers)));
    }

    @Override
    protected Path withHeading(Interpolator interpolator) {
        return new AtomicPath(curve, interpolator, modifiers);
    }

    @Override
    protected Path withModifiers(List<Modifier> modifiers) {
        return new AtomicPath(curve, interpolator, modifiers);
    }
}
