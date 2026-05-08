/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.utils;

import java.util.Objects;

public final class Pair<T, U> {
    private final T first;
    private final U second;

    private Pair(T first, U second) {
        this.first = first;
        this.second = second;
    }

    public static <T, U> Pair<T, U> of(T first, U second) {
        return new Pair<>(first, second);
    }

    public T first() {
        return first;
    }

    public U second() {
        return second;
    }

    public String toString() {
        return String.format("(%s, %s)", first, second);
    }

    public boolean equals(Object other) {
        return other instanceof Pair<?, ?>
                && Objects.equals(first, ((Pair<?, ?>) other).first)
                && Objects.equals(second, ((Pair<?, ?>) other).second);
    }

    public int hashCode() {
        return Objects.hash(first, second);
    }
}
