/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths;

import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.tvalue.TValue;
import lombok.experimental.Delegate;

public abstract class Path {
    @Delegate
    public final Curve curve;

    Path(Curve curve) {
        this.curve = curve;
    }

    abstract double getHeading(@TValue double t);
}
