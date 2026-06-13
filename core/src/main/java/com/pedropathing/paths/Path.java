/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths;

import com.pedropathing.config.Modifier;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.paths.tvalue.TValue;

import java.util.List;

import static com.pedropathing.utils.Utils.concat;
import static com.pedropathing.utils.Utils.listOf;

public abstract class Path {
    public final Curve curve;

    public final List<Modifier> modifiers;

    public Path(Curve curve, List<Modifier> modifiers) {
        this.curve = curve;
        this.modifiers = modifiers;
    }

    public abstract double heading(@TValue double t);

    public abstract List<AtomicPath> getPaths();
    public Path heading(Interpolator interpolator) {
        return withHeading(interpolator);
    }

    protected abstract Path withHeading(Interpolator interpolator);
    public Path with(List<Modifier> modifiers) {
        return withModifiers(concat(this.modifiers, modifiers));
    }
    public Path with(Modifier... modifiers) {
        return with(listOf(modifiers));
    }
    protected abstract Path withModifiers(List<Modifier> modifiers);
    public Path constant(double heading) {
        return heading(Interpolator.constant(heading));
    }

    public Path constant(Pose pose) {
        return constant(pose.heading());
    }

    public Path linear(double start, double end) {
        return heading(Interpolator.linear(start, end));
    }

    public Path linear(Pose start, Pose end) {
        return linear(start.heading(), end.heading());
    }

    public Path tangent() {
        return heading(Interpolator.tangent);
    }

    public Path facingPoint(Vector2D point) {
        return heading(Interpolator.facingPoint(point));
    }

    public Path facingPoint(Pose pose) {
        return facingPoint(pose.toVector2D());
    }
}
