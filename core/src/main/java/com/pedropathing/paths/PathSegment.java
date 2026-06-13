/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths;

import com.pedropathing.config.Modifier;
import com.pedropathing.paths.curves.Curve;
import java.util.List;

public final class PathSegment {
    public final Curve curve;
    private final HeadingProvider heading;
    private final List<Modifier> modifiers;

    PathSegment(Curve curve, HeadingProvider heading, List<Modifier> modifiers) {
        this.curve = curve;
        this.heading = heading;
        this.modifiers = modifiers;
    }

    public double heading(double t) {
        return heading.heading(t);
    }

    List<Modifier> modifiers() {
        return modifiers;
    }

    @FunctionalInterface
    public interface HeadingProvider {
        double heading(double t);
    }
}
