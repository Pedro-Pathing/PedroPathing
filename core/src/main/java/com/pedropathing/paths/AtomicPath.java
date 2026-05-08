/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths;

import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.paths.tvalue.TValue;

public abstract class AtomicPath extends Path {
    private final Interpolator interpolator;

    AtomicPath(Curve curve, Interpolator interpolator) {
        super(curve);
        this.interpolator = interpolator;
    }

    @Override
    double getHeading(@TValue double t) {
        return interpolator.interpolate(curve, t);
    }
}
