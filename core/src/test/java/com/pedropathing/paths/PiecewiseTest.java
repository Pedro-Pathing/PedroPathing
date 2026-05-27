package com.pedropathing.paths;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class PiecewiseTest {
    @Test
    public void constructorRejectsEmptyArrays() {
        assertThrows(IllegalArgumentException.class, () -> new Piecewise<>(String::length, new String[0]));
    }

    @Test
    public void segmentsAndTMappingFollowRelativeLengths() {
        Piecewise<String> piecewise = new Piecewise<>(String::length, new String[] {"aa", "bbb", "ccccc"});

        assertEquals(10.0, piecewise.length(), 1e-9);
        assertEquals(0.0, piecewise.segments().get(0).startT(), 1e-9);
        assertEquals(0.2, piecewise.segments().get(1).startT(), 1e-9);
        assertEquals(0.5, piecewise.segments().get(2).startT(), 1e-9);

        assertSame(piecewise.segments().get(0).value(), piecewise.get(0.0));
        assertSame(piecewise.segments().get(1).value(), piecewise.get(0.35));
        assertSame(piecewise.segments().get(2).value(), piecewise.get(0.9));

        assertSame(piecewise.segments().get(1), piecewise.getSegment(0.35));
        assertEquals(0.5, piecewise.localT(0.35), 1e-9);
        assertEquals(0.35, piecewise.globalT(piecewise.segments().get(1), 0.5), 1e-9);
    }
}

