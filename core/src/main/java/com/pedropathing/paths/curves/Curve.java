/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths.curves;

import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.tvalue.TValue;

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
}
