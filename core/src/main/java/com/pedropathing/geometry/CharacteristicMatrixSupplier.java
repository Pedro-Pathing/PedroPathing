package com.pedropathing.geometry;

import com.pedropathing.math.Matrix;
import com.pedropathing.math.MatrixUtil;

import java.util.HashMap;

/**
 * The CharacteristicMatrixSupplier handles supply the characteristic matrices for splines in their matrix representations.
 *
 * Splines Supported:
 * Bezier (N-degree)
 * Hermite (Cubic and Quintic)
 * Cardinal Spline
 *
 *
 * @author icaras84
 * @version 0.0.2, 07/11/2025
 */
public class CharacteristicMatrixSupplier {

    private static final HashMap<Integer, Matrix> bezierMatrices = new HashMap<>();
    private static boolean initialized = false;

    /**
     * This method sets up this class and caches some commonly used bezier curves, such as the quadratic and cubic beziers.
     */
    public void initialize(){
        if (!initialized) {
            getBezierCharacteristicMatrix(2); // quadratic bezier
            getBezierCharacteristicMatrix(3); // cubic bezier
            getBezierCharacteristicMatrix(4); // quartic bezier
            getBezierCharacteristicMatrix(5); // quintic bezier
            initialized = true;
        }
    }

    /**
     * This method is to generate Pascal's triangle (all layers being left-aligned in a matrix) without the need for factorials
     * @param layers how many layers of the triangle to generate; the minimum should be 1
     * @return Pascal's triangle (left-aligned)
     */
    private static double[][] generatePascalTriangle(int layers){
        double[][] output = new double[layers][layers];

        // pad the start and end of all layers with 1's
        for (int i = 0; i < layers; i++) {
            output[i][0] = 1;
            output[i][i] = 1;
        }

        // values in-between are calculated how Pascal's triangle layers are generated,
        // however, don't generate if the layer count is below 2
        for (int i = 2; i < layers; i++) {
            for (int j = 1; j < layers - 1; j++) {
                output[i][j] = output[i - 1][j] + output[i - 1][j - 1];
            }
        }

        return output;
    }

    /**
     * This method generates the characteristic matrix based on the degree of a requested bezier curve. 2 for quadratic, 3 for cubic, 4 for quartic, 5 for quintic... etc.
     * @param degree bezier curve's degree
     * @return characteristic matrix of the Matrix class
     */
    public static Matrix generateBezierCharacteristicMatrix(int degree){
        // get a square matrix that contains Pascal's triangle
        Matrix output = new Matrix(generatePascalTriangle(degree + 1));

        // since a bezier's characteristic matrix has rows multiplied by the n-th layer of pascal's triangle respectively
        // construct an elementary row operation matrix that does those things
        Matrix rowMultOp = new Matrix(output.getRows(), output.getColumns());
        double[] mult = output.getRow(output.getRows() - 1);

        for (int i = 0; i < rowMultOp.getRows(); i++) {
            rowMultOp.set(i, i, mult[i]);
        }

        // multiply to obtain a characteristic matrix that doesn't have the appropriate negative signs yet
        rowMultOp = rowMultOp.multiply(output);

        // flip signs if necessary
        // similar to the signs of a cofactor matrix
        int startingSign = 1;
        for (int i = 0; i < rowMultOp.getRows(); i++) {

            double[] cachedRow = rowMultOp.getRow(i);

            int realSign = startingSign;
            for (int j = 0; j < cachedRow.length; j++) {
                cachedRow[j] *= realSign;
                realSign *= -1;
            }
            rowMultOp.setRow(i, cachedRow);
            startingSign *= -1;
        }

        return new Matrix(rowMultOp);
    }

    /**
     * This method gets a characteristic matrix that is stored. If it doesn't exist, generate and return it.
     * @param degree bezier curve's degree
     * @return characteristic matrix of the Matrix class
     */
    public static Matrix getBezierCharacteristicMatrix(int degree){
        if (!bezierMatrices.containsKey(degree)){
            bezierMatrices.put(degree, generateBezierCharacteristicMatrix(degree));
        }
        return bezierMatrices.get(degree);
    }

    /**
     * This method creates a new instance of a cubic hermite spline's characteristic matrix
     * @return characteristic matrix of the Matrix class
     */
    public static Matrix generateCubicHermiteMatrix(){
        return new Matrix(new double[][]{
                { 1,  0,  0,  0},
                { 0,  1,  0,  0},
                {-3, -2,  3,  1},
                { 2,  1, -2, -1}
        });
    }

    /**
     * This method creates a new instance of a quintic hermite spline's characteristic matrix
     * @return characteristic matrix of the Matrix class
     */
    public static Matrix generateQuinticHermiteMatrix(){
        return new Matrix(new double[][]{
                {  1,  0,    0,   0,  0,   0},
                {  0,  1,    0,   0,  0,   0},
                {  0,  0,  0.5,   0,  0,   0},
                {-10, -6, -1.5,  10, -4, 0.5},
                { 15,  8,  1.5, -15,  7,  -1},
                { -6, -3, -0.5,   6, -3, 0.5}
        });
    }

    /**
     * This method creates a new instance of a cardinal spline's characteristic matrix
     * @param s tautness parameter; tension is some literature (side note: 0.5 is the catmull-rom spline)
     * @return characteristic matrix of the Matrix class
     */
    public static Matrix generateCardinalSpline(double s){
        return new Matrix(new double[][]{
                {    0,     1,         0,  0},
                {   -s,     0,         s,  0},
                {2 * s, s - 3, 3 - 2 * s, -s},
                {   -s, 2 - s,     s - 2,  s}
        });
    }
}