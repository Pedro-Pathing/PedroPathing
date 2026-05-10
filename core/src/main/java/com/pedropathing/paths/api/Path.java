/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths.api;

import com.pedropathing.config.Modifier;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.tvalue.TValue;
import lombok.experimental.Delegate;

public abstract class Path {
    @Delegate
    public final Curve curve;

    public final Modifier[] modifiers;

    Path(Curve curve, Modifier[] modifiers) {
        this.curve = curve;
        this.modifiers = modifiers;
    }

    abstract double getHeading(@TValue double t);
}
