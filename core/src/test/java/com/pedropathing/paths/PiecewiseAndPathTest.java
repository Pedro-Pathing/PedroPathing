package com.pedropathing.paths;

import com.pedropathing.config.Modifier;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.curves.Line;
import com.pedropathing.paths.interpolator.Interpolator;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

public class PiecewiseAndPathTest {
    @Test
    public void piecewiseSplitsAccordingToLengths() {
        Piecewise<String> piecewise = new Piecewise<>(String::length, new String[] {"aa", "bbb", "ccccc"});

        assertEquals(10.0, piecewise.length(), 1e-9);
        assertSame(piecewise.segments().get(0).value(), piecewise.get(0.05));
        assertSame(piecewise.segments().get(1).value(), piecewise.get(0.35));
        assertSame(piecewise.segments().get(2).value(), piecewise.get(0.8));
        assertSame(piecewise.segments().get(1), piecewise.getSegment(0.35));
        assertEquals(0.2, piecewise.segments().get(1).startT(), 1e-9);
        assertEquals(0.5, piecewise.segments().get(2).startT(), 1e-9);
        assertEquals(0.5, piecewise.localT(0.35), 1e-9);
        assertEquals(0.35, piecewise.globalT(piecewise.segments().get(1), 0.5), 1e-9);
    }

    @Test
    public void simplePathDelegatesCurveAndHeading() {
        SimplePath path = new SimplePath(
                new Line(Vector2D.cartesian(0.0, 0.0), Vector2D.cartesian(10.0, 0.0)),
                Interpolator.constant(Math.PI / 3));

        assertEquals(10.0, path.length(), 1e-9);
        assertEquals(Math.PI / 3, path.heading(0.25), 1e-9);
        List<AtomicPath> atomicPaths = path.getPaths();
        assertEquals(1, atomicPaths.size());
        assertSame(path, atomicPaths.get(0));
        assertEquals(0, path.modifiers.length);
    }

    @Test
    public void compoundPathUsesUnderlyingPathsOrExplicitInterpolator() {
        SimplePath first = new SimplePath(
                new Line(Vector2D.cartesian(0.0, 0.0), Vector2D.cartesian(10.0, 0.0)),
                Interpolator.constant(0.0));
        SimplePath second = new SimplePath(
                new Line(Vector2D.cartesian(10.0, 0.0), Vector2D.cartesian(20.0, 0.0)),
                Interpolator.constant(Math.PI / 2));

        CompoundPath compound = new CompoundPath(first, second);
        assertEquals(20.0, compound.length(), 1e-9);
        assertEquals(0.0, compound.heading(0.25), 1e-9);
        assertEquals(Math.PI / 2, compound.heading(0.75), 1e-9);

        List<AtomicPath> flattened = compound.getPaths();
        assertEquals(2, flattened.size());
        assertSame(first, flattened.get(0));
        assertSame(second, flattened.get(1));
        assertEquals(0.0, compound.segments().get(0).startT(), 1e-9);
        assertEquals(0.5, compound.segments().get(1).startT(), 1e-9);

        CompoundPath overridden = new CompoundPath(
                Interpolator.constant(Math.PI),
                new Modifier[0],
                new Path[] {first, second});
        assertEquals(Math.PI, overridden.heading(0.1), 1e-9);
        assertEquals(Math.PI, overridden.heading(0.9), 1e-9);
    }
}

