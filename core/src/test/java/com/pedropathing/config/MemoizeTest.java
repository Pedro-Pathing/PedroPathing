package com.pedropathing.config;

import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;

import static com.google.common.truth.Truth.assertThat;
import static org.junit.jupiter.api.Assertions.assertThrows;

public class MemoizeTest {
    @Test
    public void cachesResultForEqualDependencies() {
        AtomicInteger functionCalls = new AtomicInteger(0);
        Supplier<String> supplier = () -> {
            functionCalls.incrementAndGet();
            return "value";
        };
        Supplier<String> memoized = Memoize.memo(supplier, () -> "dependency");

        assertThat(memoized.get()).isEqualTo("value");
        assertThat(functionCalls.get()).isEqualTo(1);

        assertThat(memoized.get()).isEqualTo("value");
        assertThat(functionCalls.get()).isEqualTo(1);
    }

    @Test
    public void recomputesWhenDependenciesChange() {
        AtomicInteger functionCalls = new AtomicInteger(0);
        Supplier<String> supplier = () -> {
            functionCalls.incrementAndGet();
            return "value";
        };
        AtomicInteger dependency = new AtomicInteger(0);
        Supplier<String> memoized = Memoize.memo(supplier, dependency::get);

        assertThat(memoized.get()).isEqualTo("value");
        assertThat(functionCalls.get()).isEqualTo(1);

        dependency.set(1);
        assertThat(memoized.get()).isEqualTo("value");
        assertThat(functionCalls.get()).isEqualTo(2);
    }

    @Test
    public void throwsWhenNoDependencies() {
        assertThrows(IllegalArgumentException.class, () -> Memoize.memo(() -> "value"));
    }
}

