/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths;

import com.pedropathing.config.Modifier;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.tvalue.TValue;
import java.util.List;
import lombok.experimental.Delegate;

public abstract class Path {
    @Delegate
    public final Curve curve;

    public final Modifier[] modifiers;

    public Path(Curve curve, Modifier[] modifiers) {
        this.curve = curve;
        this.modifiers = modifiers;
    }

    public abstract double heading(@TValue double t);

    public abstract List<AtomicPath> getPaths();
}
