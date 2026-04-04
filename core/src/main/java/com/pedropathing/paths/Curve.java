package com.pedropathing.paths;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;

public interface Curve {
    Vector2D get(double t);
    Vector2D getDerivative(double t);
    Vector2D getSecondDerivative(double t);
    default Vector2D getTangent(double t) { return getDerivative(t).normalized(); }
    double getClosestT(Vector2D pose);

    default double curvature(double t) {
        Vector2D derivative = getDerivative(t);
        Vector2D secondDerivative = getSecondDerivative(t);
        return derivative.x * secondDerivative.y - derivative.y * secondDerivative.x / Math.pow(derivative.magnitude(), 3);
    }

    default Vector2D getNormal(double t) {
        Vector2D tangent = getTangent(t);
        return new Vector2D(-tangent.y, tangent.x);
    }

    default Vector2D endPoint() {
        return get(1.0);
    }
}
