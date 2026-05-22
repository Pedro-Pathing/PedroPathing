/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.config;

import java.util.Objects;
import java.util.function.Function;
import java.util.function.Supplier;

import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor(staticName = "memo")
public final class Memoize<T, U> implements Supplier<U> {
    private final Supplier<T> supplier;
    private final Function<T, U> function;
    private T cachedSource;
    private U cachedValue;

    @Override
    public U get() {
        T source = supplier.get();
        if (Objects.equals(source, cachedSource)) return cachedValue;
        else {
            cachedSource = source;
            return cachedValue = function.apply(source);
        }
    }
}
