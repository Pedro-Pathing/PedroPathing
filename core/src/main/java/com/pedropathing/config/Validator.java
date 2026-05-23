package com.pedropathing.config;

/**
 * @author jjophoven
 */

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
}
