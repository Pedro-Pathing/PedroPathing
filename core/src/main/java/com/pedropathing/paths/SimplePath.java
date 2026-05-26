/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths;

import com.pedropathing.config.Modifier;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.interpolator.Interpolator;

public class SimplePath extends AtomicPath {
    public SimplePath(Curve curve, Interpolator interpolator) {
        this(curve, interpolator, new Modifier[0]);
    }

    public SimplePath(Curve curve, Interpolator interpolator, Modifier[] modifiers) {
        super(curve, interpolator, modifiers);
    }
}

