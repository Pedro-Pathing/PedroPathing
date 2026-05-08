/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.config;

public class ConfigVar<T> {
    private T value;
    private boolean hasValue;

    private ConfigVar(T value) {
        this.value = value;
        this.hasValue = true;
    }

    private ConfigVar() {
        this.hasValue = false;
    }

    public static <T> ConfigVar<T> of(T value) {
        return new ConfigVar<>(value);
    }

    public static <T> ConfigVar<T> empty() {
        return new ConfigVar<>();
    }

    public T get() {
        require();
        return value;
    }

    public void set(T value) {
        this.value = value;
        this.hasValue = true;
    }

    public void require() {
        if (!hasValue) throw new IllegalStateException("Config variable has not been set");
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
}
