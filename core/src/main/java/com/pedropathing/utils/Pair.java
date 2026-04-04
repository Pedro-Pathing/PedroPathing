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
        return other instanceof Pair<?,?> &&
                first.equals(((Pair<?, ?>) other).first) &&
                second.equals(((Pair<?, ?>) other).second);
    }

    public int hashCode() {
        return Objects.hash(first, second);
    }
}
