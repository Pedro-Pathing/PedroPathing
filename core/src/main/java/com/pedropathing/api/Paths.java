/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.api;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.AtomicPath;
import com.pedropathing.paths.CompoundPath;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.curves.Line;

public final class Paths {
    private Paths() {}

    public static Path path(Path... paths) {
        return new CompoundPath(paths);
    }

    public static Path path(Curve curve) {
        return new AtomicPath(curve);
    }

    public static Path line(Vector2D start, Vector2D end) {
        return path(new Line(start, end));
    }

    public static Path line(Pose start, Pose end) {
        return path(new Line(start, end));
    }
}
