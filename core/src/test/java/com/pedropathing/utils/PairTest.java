package com.pedropathing.utils;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class PairTest {
    @Test
    public void factoryExposesStoredValues() {
        Pair<String, Integer> pair = Pair.of("left", 42);
        assertEquals("left", pair.first());
        assertEquals(42, pair.second());
    }

    @Test
    public void equalsHashCodeAndToStringUseBothValues() {
        Pair<String, Integer> pair1 = Pair.of("left", 42);
        Pair<String, Integer> pair2 = Pair.of("left", 42);
        Pair<String, Integer> different = Pair.of("right", 42);

        assertEquals(pair1, pair2);
        assertEquals(pair1.hashCode(), pair2.hashCode());
        assertNotEquals(pair1, different);
        assertEquals("(left, 42)", pair1.toString());
    }

    @Test
    public void pairHandlesNullMembers() {
        Pair<String, Integer> pair = Pair.of(null, null);
        assertNull(pair.first());
        assertNull(pair.second());
        assertEquals(Pair.of(null, null), pair);
    }
}

