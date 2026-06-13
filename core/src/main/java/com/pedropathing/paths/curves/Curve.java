/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths.curves;

import com.pedropathing.config.Modifier;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.AtomicPath;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.paths.tvalue.TValue;

import java.util.Collections;

import static java.util.Collections.emptyList;

public interface Curve {
    Vector2D get(@TValue double t);

    double closestT(Vector2D position);

    double length();

    double remainingDistance(@TValue double t);

    default double remainingDistanceNormalized(@TValue double t) {
        return remainingDistance(t) / length();
    }

    /**
     * Normalized
     */
    Vector2D tangent(@TValue double t);

    double curvature(@TValue double t);

    default Vector2D leftNormal(@TValue double t) {
        Vector2D tangent = tangent(t);
        return Vector2D.cartesian(-tangent.y(), tangent.x());
    }

    default Vector2D endPoint() {
        return get(1.0);
    }

    default Vector2D startPoint() {
        return get(0.0);
    }

    default Path heading(Interpolator interpolator) {
        return new AtomicPath(this, interpolator, emptyList());
    }

    default Path constant(double heading) {
        return heading(Interpolator.constant(heading));
    }

    default Path constant(Pose pose) {
        return constant(pose.heading());
    }

    default Path linear(double start, double end) {
        return heading(Interpolator.linear(start, end));
    }

    default Path linear(Pose start, Pose end) {
        return linear(start.heading(), end.heading());
    }

    default Path tangent() {
        return heading(Interpolator.tangent);
    }

    default Path facingPoint(Vector2D point) {
        return heading(Interpolator.facingPoint(point));
    }

    default Path facingPoint(Pose pose) {
        return facingPoint(pose.toVector2D());
    }
}
