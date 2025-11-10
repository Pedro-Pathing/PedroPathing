package com.pedropathing.math;

/**
 * This class houses static methods to create some common matrices, such as the identity matrix, zero matrix... etc.
 *
 * @author William Phomphakdee - 7462 Not to Scale Alumni
 * @author Havish Sripada - 12808 RevAmped Robotics
 * @version 1.0.0, 06/24/2025
 */
public class MatrixUtil {

    /**
     * Create a square matrix that has 1's in the diagonal while 0's everywhere else
     * @param dim row/column count of the new matrix
     * @return the identity matrix of NxN size
     */
    public static Matrix identity(int dim){
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


    /**
     * Creates a 3x3 diagonal matrix with the given diagonal entries.
     *
     * @param a00 element at (0,0)
     * @param a11 element at (1,1)
     * @param a22 element at (2,2)
     * @return a 3x3 diagonal matrix
     */
    public static Matrix diagonal3(double a00, double a11, double a22) {
        Matrix m = new Matrix(3, 3);
        m.set(0, 0, a00);
        m.set(1, 1, a11);
        m.set(2, 2, a22);
        return m;
    }

    /**
     * Inverts a 3x3 matrix.
     *
     * @param m the matrix to invert
     * @return the inverted matrix
     * @throws IllegalArgumentException if the matrix is singular
     */
    public static Matrix invert3x3(Matrix m) {
        double a = m.get(0,0), b = m.get(0,1), c = m.get(0,2);
        double d = m.get(1,0), e = m.get(1,1), f = m.get(1,2);
        double g = m.get(2,0), h = m.get(2,1), i = m.get(2,2);

        double det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
        if (Math.abs(det) < 1e-9) throw new IllegalArgumentException("Singular matrix");

        Matrix inv = new Matrix(3,3);
        inv.set(0,0, (e*i - f*h)/det);
        inv.set(0,1, (c*h - b*i)/det);
        inv.set(0,2, (b*f - c*e)/det);
        inv.set(1,0, (f*g - d*i)/det);
        inv.set(1,1, (a*i - c*g)/det);
        inv.set(1,2, (c*d - a*f)/det);
        inv.set(2,0, (d*h - e*g)/det);
        inv.set(2,1, (b*g - a*h)/det);
        inv.set(2,2, (a*e - b*d)/det);
        return inv;
    }
}