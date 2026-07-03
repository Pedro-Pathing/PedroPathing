/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths.curves;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.tvalue.TValue;
import com.pedropathing.utils.Utils;

public class Line implements Curve {
    private final Vector2D start;
    private final Vector2D end;
    private final double length;
    private final Vector2D tangent;

    public Line(Pose start, Pose end) {
        this(start.toVector2D(), end.toVector2D());
    }

    public Line(Vector2D start, Vector2D end) {
        this.start = start;
        this.end = end;
        length = start.distance(end);
        tangent = end.minus(start).normalized();
    }

    @Override
    public Vector2D startPoint() {
        return start;
    }

    @Override
    public Vector2D endPoint() {
        return end;
    }

    @Override
    public Vector2D get(@TValue double t) {
        return start.plus(end.minus(start).times(t));
    }

    @Override
    public Vector2D tangent(@TValue double t) {
        return tangent;
    }

    @Override
    public double curvature(@TValue double t) {
        return 0;
    }

    @Override
    public double closestT(Vector2D position, double initialGuess) {
        Vector2D BA = end.minus(start);
        Vector2D PA = position.minus(start);

        return Utils.clamp(BA.dot(PA) / Math.pow(BA.magnitude(), 2), 0, 1);
    }

    @Override
    public double length() {
        return length;
    }

    @Override
    public double remainingDistance(@TValue double t) {
        return (1 - t) * length();
    }

    @Override
    public double getT(double pathCompletion) {
        return pathCompletion;
    }
}
