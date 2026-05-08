/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths.curves;

import com.pedropathing.math.Vector2D;
import java.util.Arrays;

public class CompoundCurve implements Curve {
    private final Curve[] curves;
    private final double totalLength;

    public CompoundCurve(Curve... curves) {
        this.curves = curves;
        totalLength = Arrays.stream(curves).mapToDouble(Curve::length).sum();
    }

    @Override
    public Vector2D get(double t) {
        return null;
    }

    @Override
    public double closestT(Vector2D position) {
        return 0;
    }

    @Override
    public double length() {
        return totalLength;
    }

    @Override
    public Vector2D tangent(double t) {
        return null;
    }

    @Override
    public double curvature(double t) {
        return 0;
    }
}
