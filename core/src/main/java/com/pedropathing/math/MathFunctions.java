package com.pedropathing.math;

import com.pedropathing.utils.Pair;

public class MathFunctions {
    public static Pair<Double, Double> solveQuadratic(double a, double b, double c) {
        double sqrtD = Math.sqrt(b*b - 4*a*c);
        double q = -0.5 * (b + Math.copySign(sqrtD, b));
        return Pair.of(q / a, c / q);
    }
}
