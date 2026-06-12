/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.math;

import java.util.Arrays;

/**
 * A generic n-dimensional vector class.
 */
public class Vector {
    private final double[] elements;

    /**
     * Constructs a vector from an existing array.
     *
     * @param elements The values to store.
     */
    public Vector(double... elements) {
        this.elements = Arrays.copyOf(elements, elements.length);
    }

    /**
     * @return The dimensionality (length) of the vector.
     */
    public int size() {
        return elements.length;
    }

    /**
     * Gets a value at a specific index.
     *
     * @param i 0-based index.
     */
    public double get(int i) {
        return elements[i];
    }

    /**
     * Calculates the Euclidean norm (magnitude).
     */
    public double magnitude() {
        return Math.sqrt(magnitudeSquared());
    }

    public double magnitudeSquared() {
        return this.dot(this);
    }

    public Vector normalized() {
        double magnitude = magnitude();
        if (magnitude < 1e-6) throw new IllegalArgumentException("Cannot normalize 0 vector");
        return div(magnitude);
    }

    /**
     * Multiplies this vector by a scalar.
     */
    public Vector times(double scalar) {
        double[] result = Arrays.stream(elements).map(e -> e * scalar).toArray();
        return new Vector(result);
    }

    /**
     * Multiplies this vector by a scalar.
     */
    public Vector div(double scalar) {
        if (scalar == 0) throw new ArithmeticException("Cannot divide Vector by 0");
        double[] result = Arrays.stream(elements).map(e -> e / scalar).toArray();
        return new Vector(result);
    }

    /**
     * Adds another vector to this one.
     */
    public Vector plus(Vector other) {
        if (this.size() != other.size()) {
            throw new IllegalArgumentException("Vector sizes must match.");
        }
        double[] result = new double[size()];
        for (int i = 0; i < size(); i++) {
            result[i] = this.elements[i] + other.elements[i];
        }
        return new Vector(result);
    }

    /**
     * Adds another vector to this one.
     */
    public Vector minus(Vector other) {
        if (this.size() != other.size()) {
            throw new IllegalArgumentException("Vector sizes must match.");
        }
        double[] result = new double[size()];
        for (int i = 0; i < size(); i++) {
            result[i] = this.elements[i] - other.elements[i];
        }
        return new Vector(result);
    }

    /**
     * Computes the dot product of two vectors.
     */
    public double dot(Vector other) {
        if (this.size() != other.size()) {
            throw new IllegalArgumentException("Vector sizes must match.");
        }
        double sum = 0;
        for (int i = 0; i < size(); i++) {
            sum += this.elements[i] * other.elements[i];
        }
        return sum;
    }

    public Vector hadamardProduct(Vector other) {
        if (this.size() != other.size()) {
            throw new IllegalArgumentException("Vector sizes must match.");
        }

        double[] elements = new double[size()];
        for (int i = 0; i < size(); i++) {
            elements[i] = this.elements[i] * other.elements[i];
        }
        return new Vector(elements);
    }

    /**
     * Transforms this vector by a matrix (Matrix * Vector).
     * In linear algebra, this is the standard way to apply rotations, scales, or shears.
     * * @param m The transformation matrix.
     *
     * @return A new Vector resulting from the transformation.
     * @throws IllegalArgumentException if the matrix columns do not match vector size.
     */
    public Vector transform(Matrix m) {
        if (m.cols != this.size()) {
            throw new IllegalArgumentException("Matrix columns must match vector size for transformation.");
        }

        double[] result = new double[m.rows];
        for (int i = 0; i < m.rows; i++) {
            double sum = 0;
            for (int j = 0; j < m.cols; j++) {
                sum += m.get(i, j) * this.get(j);
            }
            result[i] = sum;
        }
        return new Vector(result);
    }

    public Vector2D toVector2D() {
        if (elements.length != 2) throw new IllegalArgumentException("Vector must have exactly 2 elements.");
        return Vector2D.cartesian(elements[0], elements[1]);
    }

    public Vector abs() {
        double[] elements = new double[size()];
        for (int i = 0; i < size(); i++) {
            elements[i] = Math.abs(get(i));
        }
        return new Vector(elements);
    }

    /**
     * Creates a unit vector of the specified dimensionality with a value of 1 at the specified index.
     *
     * @param i   The index at which the value is set to 1 (0-based indexing).
     * @param dim The total number of dimensions in the vector.
     * @return A unit vector with 1 at the specified index and 0 elsewhere.
     * @throws ArrayIndexOutOfBoundsException if the specified index {@code i} is out of bounds.
     */
    public static Vector e(int i, int dim) {
        double[] data = new double[dim];
        data[i] = 1;
        return new Vector(data);
    }

    public static Vector zero(int dim) {
        return new Vector(new double[dim]);
    }

    public Vector projectOnto(Vector other) {
        return other.times(dot(other) / other.dot(other));
    }

    public double quadraticForm(Matrix m) {
        return dot(transform(m));
    }

    public double angleTo(Vector other) {
        return Math.acos(dot(other) / (magnitude() * other.magnitude()));
    }

    public double distance(Vector other) {
        return minus(other).magnitude();
    }

    public Matrix toMatrix() {
        double[][] data = new double[size()][size()];
        for (int i = 0; i < size(); i++) {
            data[i][0] = elements[i];
        }
        return new Matrix(data);
    }

    public double[] elements() {
        return elements;
    }

    public boolean isZero() {
        return magnitudeSquared() < 1e-9;
    }

    public static Vector[] gramSchmidt(Vector... vectors) {
        for (int i = 0; i < vectors.length; i++) {
            vectors[i] = vectors[i].normalized();

            for (int j = i + 1; j < vectors.length; j++) {
                double comp = vectors[i].dot(vectors[j]);
                vectors[j] = vectors[j].minus(vectors[i].times(comp));
            }
        }

        return vectors;
    }
}
