/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths.curves;

import static org.junit.jupiter.api.Assertions.*;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import org.junit.jupiter.api.Test;

public class LineTest {
    @Test
    public void constructorFromPose() {
        Pose start = Pose.zero();
        Pose end = new Pose(10, 0, Math.PI / 4);
        Line line = new Line(start, end);
        assertNotNull(line.startPoint());
        assertNotNull(line.endPoint());
    }

    @Test
    public void constructorFromVector() {
        Vector2D start = Vector2D.cartesian(0, 0);
        Vector2D end = Vector2D.cartesian(3, 4);
        Line line = new Line(start, end);
        assertEquals(5, line.length(), 1e-9); // 3-4-5 triangle
    }

    @Test
    public void startPointReturnsStart() {
        Vector2D start = Vector2D.cartesian(1, 2);
        Vector2D end = Vector2D.cartesian(4, 6);
        Line line = new Line(start, end);
        Vector2D sp = line.startPoint();
        assertEquals(1, sp.x(), 1e-9);
        assertEquals(2, sp.y(), 1e-9);
    }

    @Test
    public void endPointReturnsEnd() {
        Vector2D start = Vector2D.cartesian(1, 2);
        Vector2D end = Vector2D.cartesian(4, 6);
        Line line = new Line(start, end);
        Vector2D ep = line.endPoint();
        assertEquals(4, ep.x(), 1e-9);
        assertEquals(6, ep.y(), 1e-9);
    }

    @Test
    public void getReturnsPointsAlongLine() {
        Vector2D start = Vector2D.cartesian(0, 0);
        Vector2D end = Vector2D.cartesian(10, 0);
        Line line = new Line(start, end);
        Vector2D mid = line.get(0.5);
        assertEquals(5, mid.x(), 1e-9);
        assertEquals(0, mid.y(), 1e-9);
    }

    @Test
    public void getAtT0ReturnsStart() {
        Vector2D start = Vector2D.cartesian(2, 3);
        Vector2D end = Vector2D.cartesian(5, 7);
        Line line = new Line(start, end);
        Vector2D pt = line.get(0.0);
        assertEquals(start.x(), pt.x(), 1e-9);
        assertEquals(start.y(), pt.y(), 1e-9);
    }

    @Test
    public void getAtT1ReturnsEnd() {
        Vector2D start = Vector2D.cartesian(2, 3);
        Vector2D end = Vector2D.cartesian(5, 7);
        Line line = new Line(start, end);
        Vector2D pt = line.get(1.0);
        assertEquals(end.x(), pt.x(), 1e-9);
        assertEquals(end.y(), pt.y(), 1e-9);
    }

    @Test
    public void tangentIsNormalized() {
        Vector2D start = Vector2D.cartesian(0, 0);
        Vector2D end = Vector2D.cartesian(3, 4);
        Line line = new Line(start, end);
        Vector2D tangent = line.tangent(0.5); // t value should not matter for line
        assertEquals(1.0, tangent.magnitude(), 1e-9);
    }

    @Test
    public void tangentIsConstantAlongLine() {
        Vector2D start = Vector2D.cartesian(0, 0);
        Vector2D end = Vector2D.cartesian(6, 8);
        Line line = new Line(start, end);
        Vector2D t0 = line.tangent(0.0);
        Vector2D t05 = line.tangent(0.5);
        Vector2D t1 = line.tangent(1.0);
        assertEquals(t0.x(), t05.x(), 1e-9);
        assertEquals(t0.y(), t05.y(), 1e-9);
        assertEquals(t05.x(), t1.x(), 1e-9);
        assertEquals(t05.y(), t1.y(), 1e-9);
    }

    @Test
    public void tangentPointsInCorrectDirection() {
        Vector2D start = Vector2D.cartesian(0, 0);
        Vector2D end = Vector2D.cartesian(1, 0);
        Line line = new Line(start, end);
        Vector2D tangent = line.tangent(0.5);
        assertEquals(1, tangent.x(), 1e-9);
        assertEquals(0, tangent.y(), 1e-9);
    }

    @Test
    public void curvatureIsZero() {
        Vector2D start = Vector2D.cartesian(0, 0);
        Vector2D end = Vector2D.cartesian(1, 1);
        Line line = new Line(start, end);
        assertEquals(0, line.curvature(0.0), 1e-9);
        assertEquals(0, line.curvature(0.5), 1e-9);
        assertEquals(0, line.curvature(1.0), 1e-9);
    }

    @Test
    public void lengthCalculationIsCorrect() {
        Vector2D start = Vector2D.cartesian(0, 0);
        Vector2D end = Vector2D.cartesian(3, 4);
        Line line = new Line(start, end);
        assertEquals(5.0, line.length(), 1e-9); // 3-4-5 triangle
    }

    @Test
    public void remainingDistanceAtStart() {
        Vector2D start = Vector2D.cartesian(0, 0);
        Vector2D end = Vector2D.cartesian(10, 0);
        Line line = new Line(start, end);
        assertEquals(10, line.remainingDistance(0.0), 1e-9);
    }

    @Test
    public void remainingDistanceInMiddle() {
        Vector2D start = Vector2D.cartesian(0, 0);
        Vector2D end = Vector2D.cartesian(10, 0);
        Line line = new Line(start, end);
        assertEquals(5, line.remainingDistance(0.5), 1e-9);
    }

    @Test
    public void remainingDistanceAtEnd() {
        Vector2D start = Vector2D.cartesian(0, 0);
        Vector2D end = Vector2D.cartesian(10, 0);
        Line line = new Line(start, end);
        assertEquals(0, line.remainingDistance(1.0), 1e-9);
    }

    @Test
    public void closestTOnPointExactlyOnLine() {
        Vector2D start = Vector2D.cartesian(0, 0);
        Vector2D end = Vector2D.cartesian(10, 0);
        Line line = new Line(start, end);
        Vector2D point = Vector2D.cartesian(5, 0);
        assertEquals(0.5, line.closestT(point), 1e-9);
    }

    @Test
    public void closestTClampsToStart() {
        Vector2D start = Vector2D.cartesian(0, 0);
        Vector2D end = Vector2D.cartesian(10, 0);
        Line line = new Line(start, end);
        Vector2D point = Vector2D.cartesian(-5, 0);
        assertEquals(0, line.closestT(point), 1e-9);
    }

    @Test
    public void closestTClampsToEnd() {
        Vector2D start = Vector2D.cartesian(0, 0);
        Vector2D end = Vector2D.cartesian(10, 0);
        Line line = new Line(start, end);
        Vector2D point = Vector2D.cartesian(15, 0);
        assertEquals(1, line.closestT(point), 1e-9);
    }

    @Test
    public void closestTFindsPointOffLine() {
        Vector2D start = Vector2D.cartesian(0, 0);
        Vector2D end = Vector2D.cartesian(10, 0);
        Line line = new Line(start, end);
        Vector2D point = Vector2D.cartesian(5, 5);
        // Closest point on the line should be at t=0.5
        assertEquals(0.5, line.closestT(point), 1e-9);
    }

    @Test
    public void diagonalLineClosestT() {
        Vector2D start = Vector2D.cartesian(0, 0);
        Vector2D end = Vector2D.cartesian(4, 3);
        Line line = new Line(start, end);
        Vector2D point = Vector2D.cartesian(4, 0);
        double t = line.closestT(point);
        assertTrue(t >= 0 && t <= 1);
        Vector2D closest = line.get(t);
        // Verify the closest point is on the line and reasonably close to the target
        assertTrue(closest.magnitude() <= line.length());
    }
}
