package com.pedropathing.geometry;

public interface Curve {
    Vector get(double t);
    Vector getDerivative(double t);
    Vector getSecondDerivative(double t);
    default Vector getTangent(double t) { return getDerivative(t).normalized(); }
    double getClosestT(Pose pose);
}
