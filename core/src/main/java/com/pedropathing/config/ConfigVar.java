/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.config;

import java.util.ArrayList;
import java.util.List;

public class ConfigVar<T> {
    private T value;
    private boolean hasValue;
    private final List<Validator<T>> validators = new ArrayList<>();

    private ConfigVar(T value) {
        this.value = value;
        this.hasValue = true;
    }

    private ConfigVar() {
        this.hasValue = false;
    }

    public static <T> ConfigVar<T> required() {
        return new ConfigVar<>();
    }

    public static <T> ConfigVar<T> of(T value) {
        return new ConfigVar<>(value);
    }

    public static <T> ConfigVar<T> of(T value, Validator<T> validator) {
        return new ConfigVar<>(value).validate(validator);
    }

    public T get() {
        require();
        return value;
    }

    public void set(T value) {
        this.value = value;
        validate();
        this.hasValue = true;
    }

    public Modifier modify(T tempValue) {
        return new Modifier() {
            private T originalValue;

            @Override
            public void apply() {
                originalValue = value;
                value = tempValue;
            }

            @Override
            public void revert() {
                value = originalValue;
            }
        };
    }

    public ConfigVar<T> validate(Validator<T> validator) {
        this.validators.add(validator);
        return this;
    }

    private void validate() {
        if (value == null) {
            throw new IllegalArgumentException("Config variable cannot be null");
        }
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
