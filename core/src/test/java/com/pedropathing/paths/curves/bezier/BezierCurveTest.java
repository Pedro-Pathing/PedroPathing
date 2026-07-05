/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths.curves.bezier;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.math.Vector2D;
import java.util.ArrayList;
import java.util.Arrays;
import org.junit.jupiter.api.Test;

public class BezierCurveTest {
    private static final double EPS = 1e-6;

    private static void assertPoint(Vector2D actual, double x, double y) {
        assertEquals(x, actual.x(), EPS);
        assertEquals(y, actual.y(), EPS);
    }

    @Test
    public void constructorRejectsTooFewControlPoints() {
        assertThrows(IllegalArgumentException.class,
                () -> new BezierCurve(Vector2D.cartesian(0, 0), Vector2D.cartesian(10, 10)));
    }

    @Test
    public void constructorCopiesControlPoints() {
        ArrayList<Vector2D> points = new ArrayList<>(Arrays.asList(
                Vector2D.cartesian(0, 0),
                Vector2D.cartesian(50, 100),
                Vector2D.cartesian(100, 0)));

        BezierCurve curve = new BezierCurve(points);
        points.set(1, Vector2D.cartesian(75, 75));

        assertEquals(3, curve.getControlPoints().size());
        assertPoint(curve.getControlPoints().get(0), 0, 0);
        assertPoint(curve.getControlPoints().get(1), 50, 100);
        assertPoint(curve.getControlPoints().get(2), 100, 0);
    }

    @Test
    public void quadraticStraightLineBehavesLikeLineSegment() {
        BezierCurve curve = new BezierCurve(
                Vector2D.cartesian(0, 0),
                Vector2D.cartesian(50, 0),
                Vector2D.cartesian(100, 0));

        assertAll(
                () -> assertPoint(curve.startPoint(), 0, 0),
                () -> assertPoint(curve.endPoint(), 100, 0),
                () -> assertPoint(curve.get(0.0), 0, 0),
                () -> assertPoint(curve.get(0.5), 50, 0),
                () -> assertPoint(curve.get(1.0), 100, 0),
                () -> assertEquals(100.0, curve.length(), EPS),
                () -> assertEquals(75.0, curve.remainingDistance(0.25), EPS),
                () -> assertEquals(0.75, curve.remainingDistanceNormalized(0.25), EPS),
                () -> assertEquals(0.25, curve.getPathCompletion(0.25), EPS),
                () -> assertEquals(0.25, curve.getT(0.25), EPS),
                () -> assertPoint(curve.tangent(0.5), 1, 0),
                () -> assertPoint(curve.leftNormal(0.5), 0, 1),
                () -> assertEquals(0.0, curve.curvature(0.5), EPS));
    }

    @Test
    public void quadraticArcHasExpectedDerivativeAndCurvature() {
        BezierCurve curve = new BezierCurve(
                Vector2D.cartesian(0, 0),
                Vector2D.cartesian(50, 100),
                Vector2D.cartesian(100, 0));

        Vector tVector = curve.getTVector(0.5);
        Vector firstDerivativeTVector = curve.getTVector(0.5, 1);
        Vector secondDerivativeTVector = curve.getTVector(0.5, 2);

        assertAll(
                () -> assertPoint(curve.get(0.5), 50, 50),
                () -> assertPoint(curve.derivative(0.5), 100, 0),
                () -> assertPoint(curve.getDerivative(1, 0.5), 100, 0),
                () -> assertPoint(curve.getDerivative(2, 0.5), 0, -400),
                () -> assertEquals(1.0, curve.tangent(0.5).magnitude(), EPS),
                () -> assertEquals(-0.04, curve.curvature(0.5), EPS),
                () -> assertEquals(3, tVector.size()),
                () -> assertEquals(1.0, tVector.get(0), EPS),
                () -> assertEquals(0.5, tVector.get(1), EPS),
                () -> assertEquals(0.25, tVector.get(2), EPS),
                () -> assertEquals(0.0, firstDerivativeTVector.get(0), EPS),
                () -> assertEquals(1.0, firstDerivativeTVector.get(1), EPS),
                () -> assertEquals(1.0, firstDerivativeTVector.get(2), EPS),
                () -> assertEquals(0.0, secondDerivativeTVector.get(0), EPS),
                () -> assertEquals(0.0, secondDerivativeTVector.get(1), EPS),
                () -> assertEquals(2.0, secondDerivativeTVector.get(2), EPS));
    }

    @Test
    public void closestTFindsPointOnCurveAndClampsOutsideRange() {
        BezierCurve curve = new BezierCurve(
                Vector2D.cartesian(0, 0),
                Vector2D.cartesian(50, 100),
                Vector2D.cartesian(100, 0));

        Vector2D onCurve = curve.get(0.3);

        assertAll(
                () -> assertEquals(0.3, curve.closestT(onCurve), 1e-3),
                () -> assertEquals(0.0, curve.closestT(Vector2D.cartesian(-10, 0)), EPS),
                () -> assertEquals(1.0, curve.closestT(Vector2D.cartesian(120, 0), 0.25), EPS));
    }

    @Test
    public void throughThreePointsPassesThroughMidpoint() {
        Pose[] points = new Pose[] {
            new Pose(0, 0),
            new Pose(60, 120),
            new Pose(141.5, 0)
        };

        BezierCurve curve = BezierCurve.through(points);

        assertAll(
                () -> assertPoint(curve.get(0.0), points[0].x(), points[0].y()),
                () -> assertPoint(curve.get(0.5), points[1].x(), points[1].y()),
                () -> assertPoint(curve.get(1.0), points[2].x(), points[2].y()));
    }

    @Test
    public void throughFourPointsPassesThroughAllKnots() {
        Pose[] points = new Pose[] {
            new Pose(0, 0),
            new Pose(35, 110),
            new Pose(100, 130),
            new Pose(141.5, 0)
        };

        BezierCurve curve = BezierCurve.through(points);

        assertAll(
                () -> assertPoint(curve.get(0.0), points[0].x(), points[0].y()),
                () -> assertPoint(curve.get(1.0 / 3.0), points[1].x(), points[1].y()),
                () -> assertPoint(curve.get(2.0 / 3.0), points[2].x(), points[2].y()),
                () -> assertPoint(curve.get(1.0), points[3].x(), points[3].y()));
    }

    @Test
    public void throughVarargsPassesThroughAllKnots() {
        Pose[] points = new Pose[] {
            new Pose(0, 0),
            new Pose(20, 90),
            new Pose(60, 135),
            new Pose(100, 90),
            new Pose(141.5, 0)
        };

        BezierCurve curve = BezierCurve.through(points);

        assertAll(
                () -> assertPoint(curve.get(0.0), points[0].x(), points[0].y()),
                () -> assertPoint(curve.get(0.25), points[1].x(), points[1].y()),
                () -> assertPoint(curve.get(0.5), points[2].x(), points[2].y()),
                () -> assertPoint(curve.get(0.75), points[3].x(), points[3].y()),
                () -> assertPoint(curve.get(1.0), points[4].x(), points[4].y()));
    }
}
