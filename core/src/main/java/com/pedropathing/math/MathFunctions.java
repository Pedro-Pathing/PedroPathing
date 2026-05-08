/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.math;

import com.pedropathing.utils.Pair;
import lombok.experimental.UtilityClass;

@UtilityClass
public class MathFunctions {
    public Pair<Double, Double> solveQuadratic(double a, double b, double c) {
        double sqrtD = Math.sqrt(b * b - 4 * a * c);
        double q = -0.5 * (b + Math.copySign(sqrtD, b));
        return Pair.of(q / a, c / q);
    }

    public double clamp(double num, double lower, double upper) {
        return Math.max(lower, Math.min(num, upper));
    }
}
