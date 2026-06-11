/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.utils;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collector;
import java.util.stream.Collectors;

public final class Utils {
    private Utils() {
    }

    public static Pair<Double, Double> solveQuadratic(double a, double b, double c) {
        double sqrtD = java.lang.Math.sqrt(b * b - 4 * a * c);
        double q = -0.5 * (b + java.lang.Math.copySign(sqrtD, b));
        return Pair.of(q / a, c / q);
    }

    public static double clamp(double num, double lower, double upper) {
        return Math.max(lower, Math.min(num, upper));
    }

    @SafeVarargs
    public static <T> List<T> listOf(T... elements) {
        return Collections.unmodifiableList(Arrays.asList(elements));
    }

    public static <T> Collector<T, ?, List<T>> toUnmodifiableList() {
        return Collectors.collectingAndThen(Collectors.toList(), Collections::unmodifiableList);
    }

    @SafeVarargs
    public static <T> List<T> concat(List<T>... lists) {
        return Arrays.stream(lists).flatMap(List::stream).collect(toUnmodifiableList());
    }
}
