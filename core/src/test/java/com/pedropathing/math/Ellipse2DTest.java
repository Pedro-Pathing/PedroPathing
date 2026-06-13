/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.math;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

public class Ellipse2DTest {
    @Test
    public void constructorFromAxesAndTheta() {
        double[] evals = {0.25, 0.5};
        Ellipse2D e = new Ellipse2D(Math.PI / 4, evals);
        assertEquals(Math.PI / 4, e.theta, 1e-9);
        assertEquals(0.25, e.eigenvalues[0], 1e-9);
        assertEquals(0.5, e.eigenvalues[1], 1e-9);
    }

    @Test
    public void constructorThrowsOn1x1Matrix() {
        Matrix m = Matrix.diag(0.5);
        assertThrows(IllegalArgumentException.class, () -> new Ellipse2D(m));
    }

    @Test
    public void constructorThrowsOnNegativeEigenvalue() {
        Matrix m = new Matrix(new double[][] {{-0.25, 0}, {0, 0.25}});
        assertThrows(IllegalArgumentException.class, () -> new Ellipse2D(m));
    }

    @Test
    public void constructorFromThetaAndEigenvalues() {
        double[] evals = {0.25, 0.5};
        Ellipse2D e = new Ellipse2D(Math.PI / 4, evals);
        assertEquals(Math.PI / 4, e.theta, 1e-9);
        assertEquals(0.25, e.eigenvalues[0], 1e-9);
        assertEquals(0.5, e.eigenvalues[1], 1e-9);
    }

    @Test
    public void constructorFromThetaAndAxes() {
        Ellipse2D e = new Ellipse2D(Math.PI / 6, 4, 2);
        assertEquals(Math.PI / 6, e.theta, 1e-9);
        assertEquals(4, e.getMajorAxis(), 1e-6);
        assertEquals(2, e.getMinorAxis(), 1e-6);
    }

    @Test
    public void constructorFromAxes() {
        Ellipse2D e = new Ellipse2D(5, 3);
        assertEquals(5, e.getMajorAxis(), 1e-6);
        assertEquals(3, e.getMinorAxis(), 1e-6);
        assertEquals(0, e.theta, 1e-9);
    }

    @Test
    public void circleConstructor() {
        Ellipse2D circle = new Ellipse2D(2.0);
        assertEquals(2.0, circle.getMajorAxis(), 1e-6);
        assertEquals(2.0, circle.getMinorAxis(), 1e-6);
    }

    @Test
    public void interiorPointTest() {
        Ellipse2D e = new Ellipse2D(4, 2);
        Vector2D interior = Vector2D.cartesian(1, 0.5);
        Vector2D exterior = Vector2D.cartesian(5, 0);
        assertTrue(e.interiorPoint(interior));
        assertFalse(e.interiorPoint(exterior));
    }

    @Test
    public void onBoundaryTest() {
        Ellipse2D e = new Ellipse2D(4, 2);
        Vector2D onMajor = Vector2D.cartesian(4, 0);
        Vector2D onMinor = Vector2D.cartesian(0, 2);
        assertTrue(e.onBoundary(onMajor));
        assertTrue(e.onBoundary(onMinor));
    }

    @Test
    public void radiusAtAngles() {
        Ellipse2D e = new Ellipse2D(4, 2);
        double radiusMajor = e.radius(0);
        double radiusMinor = e.radius(Math.PI / 2);
        assertEquals(4, radiusMajor, 1e-6);
        assertEquals(2, radiusMinor, 1e-6);
    }

    @Test
    public void rotateChangesTheta() {
        Ellipse2D e1 = new Ellipse2D(0, 4, 2);
        Ellipse2D e2 = e1.rotate(Math.PI / 4);
        assertEquals(Math.PI / 4, e2.theta, 1e-9);
        assertEquals(4, e2.getMajorAxis(), 1e-6);
        assertEquals(2, e2.getMinorAxis(), 1e-6);
    }

    @Test
    public void interpolateWithVector() {
        Ellipse2D e = new Ellipse2D(4, 2);
        Vector2D v = Vector2D.cartesian(1, 0);
        Vector2D result = e.interpolate(v);
        assertNotNull(result);
        assertTrue(result.magnitude() > 0);
    }

    @Test
    public void interpolateWithAngle() {
        Ellipse2D e = new Ellipse2D(4, 2);
        Vector2D atMajor = e.interpolate(0);
        Vector2D atMinor = e.interpolate(Math.PI / 2);
        assertEquals(4, atMajor.magnitude(), 1e-6);
        assertEquals(2, atMinor.magnitude(), 1e-6);
    }

    @Test
    public void fromAxesWithForwardGreaterThanLateral() {
        Ellipse2D e = new Ellipse2D(10, 5);
        assertEquals(10, e.getMajorAxis(), 1e-6);
        assertEquals(5, e.getMinorAxis(), 1e-6);
    }

    @Test
    public void fromAxesWithEqualsAxis() {
        Ellipse2D e = new Ellipse2D(5, 5);
        assertEquals(5, e.getMajorAxis(), 1e-6);
        assertEquals(5, e.getMinorAxis(), 1e-6);
    }
}
