/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.api;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.CompoundPath;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.curves.Line;

import static com.pedropathing.utils.Utils.listOf;
import static java.util.Collections.emptyList;

public final class Paths {
    private Paths() {
    }

    public static Path path(Path... paths) {
        return new CompoundPath(null, emptyList(), listOf(paths));
    }

    public static Curve line(Vector2D start, Vector2D end) {
        return new Line(start, end);
    }

    public static Curve line(Pose start, Pose end) {
        return new Line(start, end);
    }
}
