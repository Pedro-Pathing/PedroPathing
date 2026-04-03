package com.pedropathing.geometry;

/**
 * Represents a mathematical matrix of doubles.
 * Provides basic linear algebra operations including addition, 
 * multiplication, and transposition.
 */
public class Matrix {
    private final int rows;
    private final int cols;
    private final double[][] data;

    /**
     * Constructs a new matrix with the specified dimensions, initialized to zero.
     * * @param rows The number of rows.
     * @param cols The number of columns.
     */
    public Matrix(int rows, int cols) {
        this.rows = rows;
        this.cols = cols;
        this.data = new double[rows][cols];
    }

    /**
     * Constructs a new matrix from an existing 2D array.
     * Performs a deep copy to ensure the internal state is encapsulated.
     * * @param data A 2D array of doubles.
     */
    public Matrix(double[][] data) {
        this.rows = data.length;
        this.cols = data[0].length;
        this.data = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            System.arraycopy(data[i], 0, this.data[i], 0, cols);
        }
    }

    /** @return The number of rows in the matrix. */
    public int getRows() { return rows; }

    /** @return The number of columns in the matrix. */
    public int getCols() { return cols; }

    /**
     * Gets the value at a specific coordinate.
     * * @param r Row index (0-based).
     * @param c Column index (0-based).
     * @return The value at the specified position.
     */
    public double get(int r, int c) {
        return data[r][c];
    }

    /**
     * Sets the value at a specific coordinate.
     * * @param r Row index (0-based).
     * @param c Column index (0-based).
     * @param value The new value to store.
     */
    public void set(int r, int c, double value) {
        data[r][c] = value;
    }

    /**
     * Performs matrix addition.
     * * @param other The matrix to add to this one.
     * @return A new Matrix representing the sum.
     * @throws IllegalArgumentException if dimensions do not match.
     */
    public Matrix add(Matrix other) {
        if (this.rows != other.rows || this.cols != other.cols) {
            throw new IllegalArgumentException("Matrix dimensions must match for addition.");
        }
        Matrix result = new Matrix(rows, cols);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result.data[i][j] = this.data[i][j] + other.data[i][j];
            }
        }
        return result;
    }

    /**
     * Performs matrix multiplication (Dot Product).
     * * @param other The matrix to multiply by.
     * @return A new Matrix representing the product.
     * @throws IllegalArgumentException if this.cols != other.rows.
     */
    public Matrix multiply(Matrix other) {
        if (this.cols != other.rows) {
            throw new IllegalArgumentException("Dimensions mismatch: Columns of A must equal Rows of B.");
        }
        Matrix result = new Matrix(this.rows, other.cols);
        for (int i = 0; i < this.rows; i++) {
            for (int j = 0; j < other.cols; j++) {
                for (int k = 0; k < this.cols; k++) {
                    result.data[i][j] += this.data[i][k] * other.data[k][j];
                }
            }
        }
        return result;
    }

    /**
     * Creates a new matrix that is the transpose of the current matrix.
     * * @return A new Matrix where rows and columns are swapped.
     */
    public Matrix transpose() {
        Matrix result = new Matrix(cols, rows);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result.data[j][i] = this.data[i][j];
            }
        }
        return result;
    }

    public static Matrix diag(double... eigenvalues) {
        Matrix result = new Matrix(eigenvalues.length, eigenvalues.length);
        for (int i = 0; i < eigenvalues.length; i++)
            result.data[i][i] = eigenvalues[i];
        return result;
    }

    public static Matrix identity(int n) {
        Matrix result = new Matrix(n, n);
        for (int i = 0; i < n; i++)
            result.data[i][i] = 1.0;
        return result;
    }

    public static Matrix zero(int n) {
        Matrix result = new Matrix(n, n);
        for (int i = 0; i < n; i++)
            result.data[i][i] = 0.0;
        return result;
    }

    /**
     * Utility method to print the matrix to the console in a readable format.
     */
    public void print() {
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                System.out.printf("%8.2f ", data[i][j]);
            }
            System.out.println();
        }
    }
}