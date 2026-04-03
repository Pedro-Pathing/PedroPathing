package com.pedropathing.geometry;

public interface Curve {
    Vector get(double t);
    Vector getDerivative(double t);
    Vector getSecondDerivative(double t);
    default Vector getTangent(double t) { return getDerivative(t).normalized(); }
    double getClosestT(Pose pose);

    default double curvature(double t) {
        Vector derivative = getDerivative(t);
        Vector secondDerivative = getSecondDerivative(t);
        return derivative.x * secondDerivative.y - derivative.y * secondDerivative.x / Math.pow(derivative.magnitude(), 3);
    }

    default Vector getNormal(double t) {
        Vector tangent = getTangent(t);
        return new Vector(-tangent.y, tangent.x);
    }

    default Vector endPoint() {
        return get(1.0);
    }
}
