/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.config;

public class IntConfigVar {
    private int value;
    private boolean hasValue;

    private IntConfigVar(int value) {
        this.value = value;
        this.hasValue = true;
    }

    private IntConfigVar() {
        this.hasValue = false;
    }

    public static IntConfigVar of(int value) {
        return new IntConfigVar(value);
    }

    public static IntConfigVar empty() {
        return new IntConfigVar();
    }

    public int get() {
        require();
        return value;
    }

    public void set(int value) {
        this.value = value;
        this.hasValue = true;
    }

    public void require() {
        if (!hasValue) throw new IllegalStateException("Config variable has not been set");
    }

    public Modifier modify(int tempValue) {
        return new Modifier() {
            private int originalValue;

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
