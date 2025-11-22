package com.pedropathing.math;

/**
 * This class houses static methods to create some common matrices, such as the identity matrix, zero matrix... etc.
 *
 * @author William Phomphakdee - 7462 Not to Scale Alumni
 * @version 1.0.0, 06/24/2025
 */
public class MatrixUtil {

    /**
     * Create a square matrix that has 1's in the diagonal while 0's everywhere else
     * @param dim row/column count of the new matrix
     * @return the identity matrix of NxN size
     */
    public static Matrix eye(int dim){
        Matrix output = new Matrix(dim, dim);
        for (int i = 0; i < dim; i++) {
            output.set(i, i, 1);
        }
        return output;
    }

    /**
     * Creates a square matrix where all elements are 0
     * @param dim row/column count of the new matrix
     * @return a zero matrix of NxN size
     */
    public static Matrix zeros(int dim){
        return new Matrix(dim, dim);
    }

    /**
     * Creates a matrix where all elements are 0's
     * @param rows number of rows
     * @param cols number of columns
     * @return zero matrix of MxN size
     */
    public static Matrix zeros(int rows, int cols){
        return new Matrix(rows, cols);
    }

    /**
     * Takes in an N length 1d array and returns a square matrix of NxN size
     * that has the diagonal elements be the passed in array values while the rest
     * of the elements are 0's
     * @param elements 1d double array
     * @return Matrix of NxN size
     */
    public static Matrix diag(double... elements){
        Matrix output = new Matrix(elements.length, elements.length);
        for (int i = 0; i < elements.length; i++) {
            output.set(i, i, elements[i]);
        }
        return output;
    }

    /**
     * Returns an affine translation matrix of 3x3 size
     * @param x x translation
     * @param y y translation
     * @return Matrix of 3x3 size
     */
    public static Matrix translation(double x, double y){
        return new Matrix(new double[][]{
                {1, 0, x},
                {0, 1, y},
                {0, 0, 1}
        });
    }

    /**
     * Create a 3x3 matrix with a 2d rotation minor matrix on the top left
     * @param angle radians; + = CCW, - = CW
     * @return 3x3 affine rotation matrix
     */
    public static Matrix createRotation(double angle){
        double sin = Math.sin(angle);
        double cos = Math.cos(angle);
        return new Matrix(new double[][]{
                {cos, -sin, 0.0},
                {sin,  cos, 0.0},
                {0.0,  0.0, 1.0}
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
}