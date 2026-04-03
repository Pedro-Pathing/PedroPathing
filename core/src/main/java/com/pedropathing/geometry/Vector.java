package com.pedropathing.geometry;

import java.util.Arrays;

/**
 * A generic n-dimensional vector class.
 */
public class Vector {
    protected final double[] elements;

    /**
     * Constructs a vector of a specific size initialized to zero.
     * @param size The dimensionality of the vector.
     */
    public Vector(int size) {
        this.elements = new double[size];
    }

    /**
     * Constructs a vector from an existing array.
     * @param elements The values to store.
     */
    public Vector(double... elements) {
        this.elements = Arrays.copyOf(elements, elements.length);
    }

    /** @return The dimensionality (length) of the vector. */
    public int size() {
        return elements.length;
    }

    /**
     * Gets a value at a specific index.
     * @param i 0-based index.
     */
    public double get(int i) {
        return elements[i];
    }

    /**
     * Sets a value at a specific index.
     * @param i 0-based index.
     * @param value The new value.
     */
    public void set(int i, double value) {
        elements[i] = value;
    }

    /**
     * Calculates the Euclidean norm (magnitude).
     */
    public double magnitude() {
        double sum = 0;
        for (double val : elements) {
            sum += val * val;
        }
        return Math.sqrt(sum);
    }

    /**
     * Multiplies this vector by a scalar.
     */
    public Vector times(double scalar) {
        double[] result = new double[size()];
        for (int i = 0; i < size(); i++) {
            result[i] = elements[i] * scalar;
        }
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
     * @return A new Vector resulting from the transformation.
     * @throws IllegalArgumentException if the matrix columns do not match vector size.
     */
    public Vector transform(Matrix m) {
        if (m.getCols() != this.size()) {
            throw new IllegalArgumentException("Matrix columns must match vector size for transformation.");
        }

        Vector result = new Vector(m.getRows());
        for (int i = 0; i < m.getRows(); i++) {
            double sum = 0;
            for (int j = 0; j < m.getCols(); j++) {
                sum += m.get(i, j) * this.get(j);
            }
            result.set(i, sum);
        }
        return result;
    }

    /**
     * Computes the tensor product (outer product) of this vector and another vector.
     * Result is a matrix where Matrix[i][j] = this[i] * other[j].
     * * @param other The vector to multiply with.
     * @return A Matrix representing the tensor product.
     */
    public Matrix tensorProduct(Vector other) {
        Matrix result = new Matrix(this.size(), other.size());
        for (int i = 0; i < this.size(); i++) {
            for (int j = 0; j < other.size(); j++) {
                result.set(i, j, this.get(i) * other.get(j));
            }
        }
        return result;
    }

    @Override
    public String toString() {
        return Arrays.toString(elements);
    }
}