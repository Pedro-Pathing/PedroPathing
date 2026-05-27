/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths.curves;

import com.pedropathing.math.Vector2D;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class CompoundCurveTest {
    @Test
    public void constructorThrowsOnEmptyArray() {
        assertThrows(IllegalArgumentException.class, () -> new CompoundCurve());
    }

    @Test
    public void constructorWithSingleCurve() {
        Line line = new Line(Vector2D.cartesian(0, 0), Vector2D.cartesian(10, 0));
        CompoundCurve cc = new CompoundCurve(line);
        assertNotNull(cc);
        assertEquals(10, cc.length(), 1e-9);
    }

    @Test
    public void constructorWithMultipleCurves() {
        Line line1 = new Line(Vector2D.cartesian(0, 0), Vector2D.cartesian(5, 0));
        Line line2 = new Line(Vector2D.cartesian(5, 0), Vector2D.cartesian(10, 0));
        CompoundCurve cc = new CompoundCurve(line1, line2);
        assertNotNull(cc);
        assertEquals(10, cc.length(), 1e-9);
    }

    @Test
    public void lengthAggregatesCurves() {
        Line line1 = new Line(Vector2D.cartesian(0, 0), Vector2D.cartesian(3, 4));
        Line line2 = new Line(Vector2D.cartesian(3, 4), Vector2D.cartesian(6, 8));
        CompoundCurve cc = new CompoundCurve(line1, line2);
        assertEquals(10, cc.length(), 1e-9); // both are 5 units (3-4-5 triangle)
    }

    @Test
    public void startPointReturnsFirstCurveStart() {
        Line line1 = new Line(Vector2D.cartesian(1, 2), Vector2D.cartesian(5, 6));
        Line line2 = new Line(Vector2D.cartesian(5, 6), Vector2D.cartesian(9, 10));
        CompoundCurve cc = new CompoundCurve(line1, line2);
        Vector2D sp = cc.startPoint();
        assertEquals(1, sp.x(), 1e-9);
        assertEquals(2, sp.y(), 1e-9);
    }

    @Test
    public void endPointReturnsLastCurveEnd() {
        Line line1 = new Line(Vector2D.cartesian(0, 0), Vector2D.cartesian(5, 0));
        Line line2 = new Line(Vector2D.cartesian(5, 0), Vector2D.cartesian(10, 5));
        CompoundCurve cc = new CompoundCurve(line1, line2);
        Vector2D ep = cc.endPoint();
        assertEquals(10, ep.x(), 1e-9);
        assertEquals(5, ep.y(), 1e-9);
    }

    @Test
    public void getTContinuityAcrossCurves() {
        Line line1 = new Line(Vector2D.cartesian(0, 0), Vector2D.cartesian(10, 0));
        Line line2 = new Line(Vector2D.cartesian(10, 0), Vector2D.cartesian(20, 0));
        CompoundCurve cc = new CompoundCurve(line1, line2);
        // Verify that T values produce points on the line
        Vector2D pt0 = cc.get(0.0);
        Vector2D pt1 = cc.get(1.0);
        assertEquals(0, pt0.x(), 1e-9);
        assertEquals(20, pt1.x(), 1e-9);
    }

    @Test
    public void getTInMiddleProducesValidPoint() {
        Line line1 = new Line(Vector2D.cartesian(0, 0), Vector2D.cartesian(10, 0));
        Line line2 = new Line(Vector2D.cartesian(10, 0), Vector2D.cartesian(20, 0));
        CompoundCurve cc = new CompoundCurve(line1, line2);
        // T values between 0 and 1 should produce valid points
        Vector2D mid = cc.get(0.5);
        assertTrue(mid.x() >= 0 && mid.x() <= 20);
    }

    @Test
    public void getTBoundaryPoint() {
        Line line1 = new Line(Vector2D.cartesian(0, 0), Vector2D.cartesian(10, 0));
        Line line2 = new Line(Vector2D.cartesian(10, 0), Vector2D.cartesian(20, 0));
        CompoundCurve cc = new CompoundCurve(line1, line2);
        // Get a point very close to end to verify continuity
        Vector2D pt_near_end = cc.get(0.99);
        Vector2D pt_end = cc.get(1.0);
        assertTrue(pt_near_end.distance(pt_end) < 1.0);
    }

    @Test
    public void closestTFindsPointOnFirstCurve() {
        Line line1 = new Line(Vector2D.cartesian(0, 0), Vector2D.cartesian(10, 0));
        Line line2 = new Line(Vector2D.cartesian(10, 0), Vector2D.cartesian(20, 0));
        CompoundCurve cc = new CompoundCurve(line1, line2);
        double t = cc.closestT(Vector2D.cartesian(5, 0));
        assertTrue(t >= 0 && t <= 0.5);
    }

    @Test
    public void closestTFindsPointOnSecondCurve() {
        Line line1 = new Line(Vector2D.cartesian(0, 0), Vector2D.cartesian(10, 0));
        Line line2 = new Line(Vector2D.cartesian(10, 0), Vector2D.cartesian(20, 0));
        CompoundCurve cc = new CompoundCurve(line1, line2);
        double t = cc.closestT(Vector2D.cartesian(15, 0));
        assertTrue(t >= 0.5 && t <= 1.0);
    }

    @Test
    public void remainingDistanceDecreases() {
        Line line1 = new Line(Vector2D.cartesian(0, 0), Vector2D.cartesian(5, 0));
        Line line2 = new Line(Vector2D.cartesian(5, 0), Vector2D.cartesian(10, 0));
        CompoundCurve cc = new CompoundCurve(line1, line2);
        double rd_start = cc.remainingDistance(0.0);
        double rd_mid = cc.remainingDistance(0.5);
        double rd_end = cc.remainingDistance(1.0);
        assertTrue(rd_start >= rd_mid);
        assertTrue(rd_mid >= rd_end);
    }

    @Test
    public void remainingDistanceNonNegative() {
        Line line1 = new Line(Vector2D.cartesian(0, 0), Vector2D.cartesian(5, 0));
        Line line2 = new Line(Vector2D.cartesian(5, 0), Vector2D.cartesian(10, 0));
        CompoundCurve cc = new CompoundCurve(line1, line2);
        assertTrue(cc.remainingDistance(0.0) >= 0);
        assertTrue(cc.remainingDistance(0.5) >= 0);
        assertTrue(cc.remainingDistance(1.0) >= 0);
    }


    @Test
    public void tangentFromFirstCurve() {
        Line line1 = new Line(Vector2D.cartesian(0, 0), Vector2D.cartesian(10, 0));
        Line line2 = new Line(Vector2D.cartesian(10, 0), Vector2D.cartesian(20, 0));
        CompoundCurve cc = new CompoundCurve(line1, line2);
        Vector2D tangent = cc.tangent(0.25);
        assertEquals(1, tangent.magnitude(), 1e-9);
        assertEquals(1, tangent.x(), 1e-9); // pointing right
        assertEquals(0, tangent.y(), 1e-9);
    }

    @Test
    public void tangentFromSecondCurve() {
        Line line1 = new Line(Vector2D.cartesian(0, 0), Vector2D.cartesian(10, 0));
        Line line2 = new Line(Vector2D.cartesian(10, 0), Vector2D.cartesian(20, 0));
        CompoundCurve cc = new CompoundCurve(line1, line2);
        Vector2D tangent = cc.tangent(0.75);
        assertEquals(1, tangent.magnitude(), 1e-9);
        assertEquals(1, tangent.x(), 1e-9); // pointing right
        assertEquals(0, tangent.y(), 1e-9);
    }

    @Test
    public void curvatureLineSegments() {
        Line line1 = new Line(Vector2D.cartesian(0, 0), Vector2D.cartesian(10, 0));
        Line line2 = new Line(Vector2D.cartesian(10, 0), Vector2D.cartesian(20, 0));
        CompoundCurve cc = new CompoundCurve(line1, line2);
        assertEquals(0, cc.curvature(0.25), 1e-9);
        assertEquals(0, cc.curvature(0.5), 1e-9);
        assertEquals(0, cc.curvature(0.75), 1e-9);
    }

    @Test
     public void leftNormalOnFirstCurve() {
         Line line1 = new Line(Vector2D.cartesian(0, 0), Vector2D.cartesian(10, 0));
         Line line2 = new Line(Vector2D.cartesian(10, 0), Vector2D.cartesian(20, 0));
         CompoundCurve cc = new CompoundCurve(line1, line2);
         Vector2D normal = cc.leftNormal(0.25);
         assertEquals(1, normal.magnitude(), 1e-9);
         assertEquals(0, normal.x(), 1e-9); // perpendicular to rightward tangent
         assertEquals(1, normal.y(), 1e-9); // pointing left-up
     }
}


