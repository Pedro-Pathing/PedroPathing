package com.pedropathing.geometry;

public interface Curve {
    Vector2D get(double t);
    Vector2D getDerivative(double t);
    Vector2D getSecondDerivative(double t);
    default Vector2D getTangent(double t) {
        Vector2D deriv = getDerivative(t);
        if (deriv.isZero())
            throw new IllegalArgumentException("Cannot follow degenerate curve");
        return deriv.normalized();
    }
    double getClosestT(Pose pose);

    default double curvature(double t) {
        Vector2D derivative = getDerivative(t);
        Vector2D secondDerivative = getSecondDerivative(t);
        return derivative.x * secondDerivative.y - derivative.y * secondDerivative.x / Math.pow(derivative.magnitude(), 3);
    }

    default Vector2D leftGradient(double t) {
        Vector2D tangent = getTangent(t);
        return tangent.rotate(-Math.PI / 2);
    }

    default Vector2D rightGradient(double t) {
        Vector2D tangent = getTangent(t);
        return tangent.rotate(Math.PI / 2);
    }

    default Vector2D principalNormal(double t) {
        Vector2D tangent = getTangent(t);
        Vector2D secondDeriv = getSecondDerivative(t);
        Vector2D aNormal = secondDeriv.minus(tangent.times(secondDeriv.dot(tangent)));
        if (aNormal.isZero()) return Vector2D.zero();
        return aNormal.normalized();
    }

    default Vector2D endPoint() {
        return get(1.0);
    }
}
