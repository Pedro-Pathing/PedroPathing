/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.math;

import lombok.Value;

@Value
public class Twist {
    private static final Twist ZERO = new Twist(0, 0, 0);
    double vx;
    double vy;
    double omega;

    public static Twist zero() {
        return ZERO;
    }

    public static Twist fromVector(Vector2D vector) {
        return new Twist(vector.x(), vector.y(), 0);
    }

    public Velocity toVelocity(double heading) {
        return new Velocity(
                vx * Math.cos(heading) + vy * -Math.sin(heading),
                vx * Math.sin(heading) + vy * Math.cos(heading),
                omega);
    }

    public Vector toVector() {
        return new Vector(vx, vy, omega);
    }

    public Matrix toMatrix() {
        return new Matrix(new double[][] {
            {0.0, -omega, vx},
            {omega, 0.0, vy},
            {0.0, 0.0, 0.0}
        });
    }

    public Twist plus(Twist other) {
        return new Twist(vx + other.vx, vy + other.vy, omega + other.omega);
    }

    public Twist times(double scalar) {
        return new Twist(vx * scalar, vy * scalar, omega * scalar);
    }

    public Vector2D toLinear() {
        return Vector2D.cartesian(vx, vy);
    }
}
