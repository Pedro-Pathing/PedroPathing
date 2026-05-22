/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths.curves;

import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.PathProgress;
import com.pedropathing.paths.tvalue.TValue;

public interface Curve {
    Vector2D get(@TValue double t);

    double closestT(Vector2D position);

    double length();

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

    default double distanceRemaining(@TValue double t) {
        // t is proportional to arc length along the curve
        return length() * (1 - t);
    }

    // TODO it might be better to have t value not clamped to [0,1] for checking before parametric start
    default double displacementToPoint(Vector2D pathPoint, Vector2D currentPosition) {
        return pathPoint
                .minus(currentPosition)
                .dot(tangent(closestT(currentPosition)));
    }

    default double displacementToStart(Vector2D currentPosition) {
        return displacementToPoint(startPoint(), currentPosition);
    }
    default PathProgress progressAt(Vector2D position) {
        return PathProgress.at(this, closestT(position));
    }
}
