/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.config;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import static com.pedropathing.utils.Utils.listOf;

public final class Memoize<T> implements Supplier<T> {
    private final Supplier<T> supplier;
    private final List<Supplier<?>> dependencies;
    private T cachedValue;
    private List<?> cachedDependencies;

    private Memoize(Supplier<T> supplier, List<Supplier<?>> dependencies) {
        this.supplier = supplier;
        this.dependencies = dependencies;
    }

    public static <T> Memoize<T> memo(Supplier<T> supplier, List<Supplier<?>> dependencies) {
        if (dependencies.isEmpty()) throw new IllegalArgumentException("Memoize requires at least one dependency");
        return new Memoize<>(supplier, Collections.unmodifiableList(dependencies));
    }

    public static <T> Memoize<T> memo(Supplier<T> supplier, Supplier<?>... dependencies) {
        return memo(supplier, listOf(dependencies));
    }

    @Override
    public T get() {
        List<?> dependencies = this.dependencies.stream().map(Supplier::get).collect(Collectors.toList());
        if (cachedDependencies == null || !cachedDependencies.equals(dependencies)) {
            cachedValue = supplier.get();
            cachedDependencies = dependencies;
        }
        return cachedValue;
    }
}
