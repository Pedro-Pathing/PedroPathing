/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.math;

import com.pedropathing.utils.Pair;

/**
 * Represents a mathematical matrix of doubles.
 * Provides basic linear algebra operations including addition,
 * multiplication, and transposition.
 */
public class Matrix {
    public final int rows;
    public final int cols;
    private final double[] data;

    /**
     * Constructs a new matrix from an existing 2D array.
     * Performs a deep copy to ensure the internal state is encapsulated.
     *
     * @param data A 2D array of doubles.
     */
    public Matrix(double[][] data) {
        this.rows = data.length;
        this.cols = data[0].length;
        this.data = new double[rows * cols];
        for (int i = 0; i < rows; i++) {
            if (data[i].length != cols)
                throw new IllegalArgumentException(String.format(
                        "Matrix row 0 had length %d but matrix row %d had length %d", cols, i, data[i].length));
            System.arraycopy(data[i], 0, this.data, i * cols, cols);
        }
    }

    public static Matrix diag(double... eigenvalues) {
        double[][] data = new double[eigenvalues.length][eigenvalues.length];
        for (int i = 0; i < eigenvalues.length; i++) data[i][i] = eigenvalues[i];
        return new Matrix(data);
    }

    public static Matrix diag(Vector vector) {
        return diag(vector.elements());
    }

    public static Matrix diag(Vector2D vector) {
        return diag(vector.x(), vector.y());
    }

    public static Matrix identity(int n) {
        double[][] data = new double[n][n];
        for (int i = 0; i < n; i++) data[i][i] = 1;
        return new Matrix(data);
    }

    public static Matrix zero(int n) {
        double[][] data = new double[n][n];
        return new Matrix(data);
    }

    public static Matrix rotation(double theta) {
        double sin = Math.sin(theta);
        double cos = Math.cos(theta);

        return new Matrix(new double[][] {
            {cos, -sin},
            {sin, cos}
        });
    }

    /**
     * Create a 3x3 matrix with a 2d rotation minor matrix on the top left
     * @param theta radians; + = CCW, - = CW
     * @return 3x3 affine rotation matrix
     */
    public static Matrix rotationTransform(double theta) {
        double sin = Math.sin(theta);
        double cos = Math.cos(theta);
        return new Matrix(new double[][]{
                {cos, -sin, 0.0},
                {sin,  cos, 0.0},
                {0.0,  0.0, 1.0}
        });
    }

    /**
     * Returns an affine translation matrix of 3x3 size
     * @param x x translation
     * @param y y translation
     * @return Matrix of 3x3 size
     */
    public static Matrix translationTransform(double x, double y){
        return new Matrix(new double[][]{
                {1, 0, x},
                {0, 1, y},
                {0, 0, 1}
        });
    }

    /**
     * Returns an affine transformation of 3x3 matrix. This matrix represents a rotation and then a translation
     * @param x x translation
     * @param y y translation
     * @param angle radians; + = CCW, - = CW
     * @return 3x3 transformation matrix
     */
    public static Matrix createTransformation(double x, double y, double angle){
        double sin = Math.sin(angle);
        double cos = Math.cos(angle);
        return new Matrix(new double[][]{
                {cos, -sin,   x},
                {sin,  cos,   y},
                {0.0,  0.0, 1.0}
        });
    }

    /**
     * Gets the value at a specific coordinate.
     * * @param r Row index (0-based).
     *
     * @param c Column index (0-based).
     * @return The value at the specified position.
     */
    public double get(int r, int c) {
        return data[r * cols + c];
    }

    /**
     * Performs matrix addition.
     * * @param other The matrix to add to this one.
     *
     * @return A new Matrix representing the sum.
     * @throws IllegalArgumentException if dimensions do not match.
     */
    public Matrix plus(Matrix other) {
        if (this.rows != other.rows || this.cols != other.cols)
            throw new IllegalArgumentException("Matrix dimensions must match for addition.");
        double[][] data = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                data[i][j] = get(i, j) + other.get(i, j);
            }
        }
        return new Matrix(data);
    }

    /**
     * Performs matrix subtraction.
     * * @param other The matrix to subtract from this one.
     *
     * @return A new Matrix representing the sum.
     * @throws IllegalArgumentException if dimensions do not match.
     */
    public Matrix minus(Matrix other) {
        if (this.rows != other.rows || this.cols != other.cols)
            throw new IllegalArgumentException("Matrix dimensions must match for addition.");
        double[][] data = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                data[i][j] = get(i, j) - other.get(i, j);
            }
        }
        return new Matrix(data);
    }

    /**
     * Performs matrix multiplication (Dot Product).
     * * @param other The matrix to multiply by.
     *
     * @return A new Matrix representing the product.
     * @throws IllegalArgumentException if this.cols != other.rows.
     */
    public Matrix times(Matrix other) {
        if (this.cols != other.rows)
            throw new IllegalArgumentException("Dimensions mismatch: Columns of A must equal Rows of B.");
        double[][] data = new double[this.rows][other.cols];
        for (int i = 0; i < this.rows; i++) {
            for (int j = 0; j < other.cols; j++) {
                for (int k = 0; k < this.cols; k++) {
                    data[i][j] += get(i, k) * other.get(k, j);
                }
            }
        }
        return new Matrix(data);
    }

    /**
     * Creates a new matrix that is the transpose of the current matrix.
     * * @return A new Matrix where rows and columns are swapped.
     */
    public Matrix transpose() {
        double[][] data = new double[cols][rows];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                data[j][i] = get(i, j);
            }
        }
        return new Matrix(data);
    }

    /**
     * Multiplies this matrix by any n-dimensional Vector.
     */
    public Vector times(Vector v) {
        if (this.cols != v.size()) throw new IllegalArgumentException("Dimension mismatch");

        double[] result = new double[this.rows];
        for (int i = 0; i < this.rows; i++) {
            double sum = 0;
            for (int j = 0; j < this.cols; j++) {
                sum += this.get(i, j) * v.get(j);
            }
            result[i] = sum;
        }
        return new Vector(result);
    }

    public Vector getDiagonal() {
        double[] elements = new double[Math.min(rows, cols)];
        for (int i = 0; i < elements.length; i++) {
            elements[i] = get(i, i);
        }
        return new Vector(elements);
    }

    public Matrix clampDiagonals(double epsilon) {
        double[][] data = new double[rows][cols];

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (i != j) data[i][j] = get(i, j);
                else data[i][j] = Math.max(epsilon, get(i, j));
            }
        }

        return new Matrix(data);
    }

    public Pair<Matrix, Matrix> rref(Matrix augment) {
        throw new UnsupportedOperationException("i graciously decline to work");
    }

    public Matrix invert() {
        throw new UnsupportedOperationException("we will do it at some point");
    }
}
