/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths.curves;

import com.pedropathing.math.Vector2D;

public interface Curve {
    Vector2D get(double t);

    double closestT(Vector2D position, double initialGuess);

    default double closestT(Vector2D position) {
        return closestT(position, 0.5);
    }

    double length();

    double remainingDistance(double t);

    default double remainingDistanceNormalized(double t) {
        return remainingDistance(t) / length();
    }

    double getT(double pathCompletion);

    Vector2D derivative(double t);

    /**
     * Normalized
     */
    default Vector2D tangent(double t) {
        return derivative(t).normalized();
    }

    double curvature(double t);

    default Vector2D leftNormal(double t) {
        Vector2D tangent = tangent(t);
        return Vector2D.cartesian(-tangent.y(), tangent.x());
    }

    default Vector2D endPoint() {
        return get(1.0);
    }

    default Vector2D startPoint() {
        return get(0.0);
    }
}
