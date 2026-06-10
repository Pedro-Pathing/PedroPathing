/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.config;

import java.util.Objects;

/**
 * @author jjophoven
 */
@FunctionalInterface
public interface Validator<T> {
    boolean validate(T value);

    static Validator<Double> positive() {
        return v -> v > 0;
    }

    static Validator<Double> nonnegative() {
        return v -> v >= 0;
    }

    static Validator<Double> negative() {
        return v -> v < 0;
    }

    static <T> Validator<T> nonnull() {
        return Objects::nonNull;
    }
}
