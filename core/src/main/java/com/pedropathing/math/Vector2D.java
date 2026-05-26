/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.math;

import lombok.Value;

@Value(staticConstructor = "cartesian")
public class Vector2D {
    private static final Vector2D ZERO = new Vector2D(0, 0);
    private static final Vector2D I_HAT = new Vector2D(1, 0);
    private static final Vector2D J_HAT = new Vector2D(0, 1);
    double x;
    double y;

    public static Vector2D polar(double radius, double angle) {
        return new Vector2D(radius * Math.cos(angle), radius * Math.sin(angle));
    }

    public static Vector2D unit(double angle) {
        return polar(1, angle);
    }

    public static Vector2D zero() {
        return ZERO;
    }

    public static Vector2D iHat() {
        return I_HAT;
    }

    public static Vector2D jHat() {
        return J_HAT;
    }

    public double magnitude() {
        return Math.hypot(x, y);
    }

    public Vector2D normalized() {
        double magnitude = magnitude();
        if (Math.abs(magnitude) < 1e-6) throw new IllegalArgumentException("Cannot normalize 0 vector");
        return this.div(magnitude);
    }

    public Vector2D plus(Vector2D other) {
        return new Vector2D(x + other.x, y + other.y);
    }

    public Vector2D minus(Vector2D other) {
        return new Vector2D(x - other.x, y - other.y);
    }

    public Vector2D times(double scalar) {
        return new Vector2D(x * scalar, y * scalar);
    }

    public Vector2D div(double scalar) {
        return new Vector2D(x / scalar, y / scalar);
    }

    public Vector2D rotate(double angle) {
        return new Vector2D(x * Math.cos(angle) - y * Math.sin(angle), x * Math.sin(angle) + y * Math.cos(angle));
    }

    public double dot(Vector2D other) {
        return x * other.x + y * other.y;
    }

    public Vector2D cross(Vector2D other) {
        return new Vector2D(x * other.y - y * other.x, x * other.x + y * other.y);
    }

    public double det(Vector2D other) {
        return x * other.y - y * other.x;
    }

    public Vector2D projectOnto(Vector2D other) {
        return other.times(dot(other) / other.dot(other));
    }

    public double theta() {
        return Math.atan2(y, x);
    }

    public Vector toVector() {
        return new Vector(x, y);
    }

    public Vector2D transform(Matrix m) {
        return this.toVector().transform(m).toVector2D();
    }

    public boolean isZero() {
        return x < 1e-9 && y < 1e-9;
    }

    public double quadraticForm(Matrix m) {
        return dot(transform(m));
    }

    public double angleTo(Vector2D other) {
        return Math.acos(dot(other) / (magnitude() * other.magnitude()));
    }

    public double distance(Vector2D other) {
        return Math.hypot(x - other.x, y - other.y);
    }

    public Vector2D hadamardProduct(Vector2D other) {
        return new Vector2D(this.x * other.x, this.y * other.y);
    }

    public Vector2D perpendicularLeft() {
        return new Vector2D(-y, x);
    }

    public Pose toPose(double heading) {
        return new Pose(x, y, heading);
    }

    public Pose toPose() {
        return new Pose(x, y, 0);
    }
}
