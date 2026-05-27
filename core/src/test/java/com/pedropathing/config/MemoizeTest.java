package com.pedropathing.config;

import org.junit.jupiter.api.Test;

import java.util.Locale;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;

import static org.junit.jupiter.api.Assertions.*;

public class MemoizeTest {
    @Test
    public void memoizeCachesResultsForEqualSources() {
        AtomicInteger functionCalls = new AtomicInteger();
        Supplier<String> source = () -> new String("alpha");
        Memoize<String, Integer> memoize = Memoize.memo(source, value -> {
            functionCalls.incrementAndGet();
            return value.length();
        });

        assertEquals(5, memoize.get());
        assertEquals(5, memoize.get());
        assertEquals(1, functionCalls.get());
    }

    @Test
    public void memoizeRecomputesWhenSourceChanges() {
        AtomicInteger index = new AtomicInteger();
        AtomicInteger functionCalls = new AtomicInteger();
        String[] values = {"alpha", "alpha", "beta", "beta"};
        Memoize<String, String> memoize = Memoize.memo(() -> values[index.getAndIncrement()], value -> {
            functionCalls.incrementAndGet();
            return value.toUpperCase(Locale.ROOT);
        });

        assertEquals("ALPHA", memoize.get());
        assertEquals("ALPHA", memoize.get());
        assertEquals("BETA", memoize.get());
        assertEquals("BETA", memoize.get());
        assertEquals(2, functionCalls.get());
    }
}

