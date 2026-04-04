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
        double sum = this.dot(this);
        return Math.sqrt(sum);
    }

    /**
     * Multiplies this vector by a scalar.
     */
    public Vector times(double scalar) {
        double[] result = Arrays.stream(elements).map(e -> e * scalar).toArray();
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
        return new Vector2D(elements[0], elements[1]);
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

    public Matrix toMatrix() {
        double[][] data = new double[size()][size()];
        for (int i = 0; i < size(); i++) {
            data[i][0] = elements[i];
        }
        return new Matrix(data);
    }
}