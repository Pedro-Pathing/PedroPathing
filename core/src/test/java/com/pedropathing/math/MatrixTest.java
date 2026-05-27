/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.math;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class MatrixTest {
    @Test
    public void constructorStoresAndAccessesData() {
        double[][] data = {{1, 2}, {3, 4}};
        Matrix m = new Matrix(data);
        assertEquals(2, m.rows);
        assertEquals(2, m.cols);
        assertEquals(1, m.get(0, 0));
        assertEquals(2, m.get(0, 1));
        assertEquals(3, m.get(1, 0));
        assertEquals(4, m.get(1, 1));
    }

    @Test
    public void constructorThrowsOnMismatchedRowLengths() {
        double[][] data = {{1, 2}, {3, 4, 5}};
        assertThrows(IllegalArgumentException.class, () -> new Matrix(data));
    }

    @Test
    public void diagCreatesInverse() {
        Matrix d = Matrix.diag(2, 3, 4);
        assertEquals(3, d.rows);
        assertEquals(3, d.cols);
        assertEquals(2, d.get(0, 0));
        assertEquals(3, d.get(1, 1));
        assertEquals(4, d.get(2, 2));
        assertEquals(0, d.get(0, 1));
        assertEquals(0, d.get(1, 0));
    }

    @Test
    public void identityCreatesIdentity() {
        Matrix i = Matrix.identity(3);
        assertEquals(3, i.rows);
        assertEquals(3, i.cols);
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                assertEquals(r == c ? 1.0 : 0.0, i.get(r, c));
            }
        }
    }

    @Test
    public void zeroCreatesZero() {
        Matrix z = Matrix.zero(2);
        assertEquals(2, z.rows);
        assertEquals(2, z.cols);
        for (int r = 0; r < 2; r++) {
            for (int c = 0; c < 2; c++) {
                assertEquals(0.0, z.get(r, c));
            }
        }
    }

    @Test
    public void rotationMatrixRotates90Degrees() {
        Matrix rot = Matrix.rotation(Math.PI / 2);
        assertEquals(0, rot.get(0, 0), 1e-9); // cos(90) ≈ 0
        assertEquals(-1, rot.get(0, 1), 1e-9); // -sin(90) = -1
        assertEquals(1, rot.get(1, 0), 1e-9); // sin(90) = 1
        assertEquals(0, rot.get(1, 1), 1e-9); // cos(90) ≈ 0
    }

    @Test
    public void matrixAddition() {
        Matrix a = new Matrix(new double[][]{{1, 2}, {3, 4}});
        Matrix b = new Matrix(new double[][]{{5, 6}, {7, 8}});
        Matrix sum = a.plus(b);
        assertEquals(6, sum.get(0, 0));
        assertEquals(8, sum.get(0, 1));
        assertEquals(10, sum.get(1, 0));
        assertEquals(12, sum.get(1, 1));
    }

    @Test
    public void matrixAdditionThrowsOnMismatchedDimensions() {
        Matrix a = new Matrix(new double[][]{{1, 2}, {3, 4}});
        Matrix b = new Matrix(new double[][]{{1, 2, 3}, {4, 5, 6}});
        assertThrows(IllegalArgumentException.class, () -> a.plus(b));
    }

    @Test
    public void matrixMultiplication() {
        Matrix a = new Matrix(new double[][]{{1, 2}, {3, 4}});
        Matrix b = new Matrix(new double[][]{{5, 6}, {7, 8}});
        Matrix prod = a.times(b);
        assertEquals(19, prod.get(0, 0)); // 1*5 + 2*7
        assertEquals(22, prod.get(0, 1)); // 1*6 + 2*8
        assertEquals(43, prod.get(1, 0)); // 3*5 + 4*7
        assertEquals(50, prod.get(1, 1)); // 3*6 + 4*8
    }

    @Test
    public void matrixMultiplicationThrowsOnMismatchedDimensions() {
        Matrix a = new Matrix(new double[][]{{1, 2}, {3, 4}});
        Matrix b = new Matrix(new double[][]{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}});
        assertThrows(IllegalArgumentException.class, () -> a.times(b));
    }

    @Test
    public void transpose() {
        Matrix m = new Matrix(new double[][]{{1, 2, 3}, {4, 5, 6}});
        Matrix t = m.transpose();
        assertEquals(3, t.rows);
        assertEquals(2, t.cols);
        assertEquals(1, t.get(0, 0));
        assertEquals(4, t.get(0, 1));
        assertEquals(2, t.get(1, 0));
        assertEquals(5, t.get(1, 1));
        assertEquals(3, t.get(2, 0));
        assertEquals(6, t.get(2, 1));
    }

    @Test
    public void matrixTimesVector() {
        Matrix m = new Matrix(new double[][]{{1, 2}, {3, 4}});
        Vector v = new Vector(5, 6);
        Vector result = m.times(v);
        assertEquals(2, result.size());
        assertEquals(17, result.get(0)); // 1*5 + 2*6
        assertEquals(39, result.get(1)); // 3*5 + 4*6
    }

    @Test
    public void matrixTimesVectorThrowsOnMismatchedDimensions() {
        Matrix m = new Matrix(new double[][]{{1, 2}, {3, 4}});
        Vector v = new Vector(5, 6, 7);
        assertThrows(IllegalArgumentException.class, () -> m.times(v));
    }

    @Test
    public void identityTimesVectorReturnsVector() {
        Matrix i = Matrix.identity(3);
        Vector v = new Vector(1, 2, 3);
        Vector result = i.times(v);
        assertEquals(1, result.get(0));
        assertEquals(2, result.get(1));
        assertEquals(3, result.get(2));
    }
}

