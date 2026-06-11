/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths;

import com.pedropathing.config.Modifier;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.paths.tvalue.TValue;
import java.util.Collections;
import java.util.List;

public class AtomicPath extends Path {
    private final Interpolator interpolator;

    public AtomicPath(Curve curve, Interpolator interpolator, Modifier[] modifiers) {
        super(curve, modifiers);
        this.interpolator = interpolator;
    }

    @Override
    public double heading(@TValue double t) {
        return interpolator.interpolate(curve, t);
    }

    @Override
    public List<AtomicPath> getPaths() {
        return Collections.singletonList(this);
    }
}
