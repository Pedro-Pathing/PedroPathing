/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.config;

import static com.pedropathing.utils.Utils.concat;
import static com.pedropathing.utils.Utils.listOf;

import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

public class ConfigVar<T> implements Supplier<T> {
    private final List<Validator<T>> validators;
    private T value;
    private boolean hasValue;

    private ConfigVar(T value, List<Validator<T>> validators) {
        this.value = value;
        this.hasValue = true;
        this.validators = Collections.unmodifiableList(validators);
        validate(value);
    }

    private ConfigVar(List<Validator<T>> validators) {
        this.hasValue = false;
        this.validators = Collections.unmodifiableList(validators);
    }

    public static <T> ConfigVar<T> required(List<Validator<T>> validators) {
        return new ConfigVar<>(concat(validators, Collections.singletonList(Validator.nonnull())));
    }

    @SafeVarargs
    public static <T> ConfigVar<T> required(Validator<T>... validators) {
        return required(listOf(validators));
    }

    public static <T> ConfigVar<T> requiredNullable(List<Validator<T>> validators) {
        return new ConfigVar<>(validators);
    }

    @SafeVarargs
    public static <T> ConfigVar<T> requiredNullable(Validator<T>... validators) {
        return requiredNullable(listOf(validators));
    }

    public static <T> ConfigVar<T> of(T value, List<Validator<T>> validators) {
        return new ConfigVar<>(value, concat(validators, Collections.singletonList(Validator.nonnull())));
    }

    @SafeVarargs
    public static <T> ConfigVar<T> of(T value, Validator<T>... validators) {
        return of(value, listOf(validators));
    }

    public static <T> ConfigVar<T> ofNullable(T value, List<Validator<T>> validators) {
        return new ConfigVar<>(value, validators);
    }

    @SafeVarargs
    public static <T> ConfigVar<T> ofNullable(T value, Validator<T>... validators) {
        return ofNullable(value, listOf(validators));
    }

    @Override
    public T get() {
        require();
        validate(value);
        return value;
    }

    public void set(T value) {
        this.value = value;
        this.hasValue = true;
    }

    public Modifier at(T tempValue) {
        validate(tempValue);
        return new Modifier() {
            private T originalValue;

            @Override
            public void apply() {
                require();
                originalValue = value;
                value = tempValue;
            }

            @Override
            public void revert() {
                value = originalValue;
            }
        };
    }

    private void validate(T value) {
        for (Validator<T> validator : validators) {
            if (!validator.validate(value)) {
                throw new IllegalArgumentException("Invalid value for config variable of " + value);
            }
        }
    }

    private void require() {
        if (!hasValue) throw new IllegalStateException("Config variable has not been set");
    }
}
