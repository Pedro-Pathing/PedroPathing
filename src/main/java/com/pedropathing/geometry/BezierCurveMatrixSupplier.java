package com.pedropathing.geometry;

import com.pedropathing.math.Matrix;

import java.util.HashMap;

/**
 * The BezierCurveMatrixSupplier class is responsible for "caching" characteristic matrices of bezier curves.
 * If necessary, this class if capable of generating characteristic matrices dynamically. Characteristic matrices
 * allows for the bezier curve to pre-multiply with a matrix of control points to allow for caching and easier operations (like differentiation).
 * If control points change, the computation has to happen again.
 *
 * @author icaras84
 * @version 0.0.1 06/14/2025
 */
public class BezierCurveMatrixSupplier {

    private static final HashMap<Integer, Matrix> matrices = new HashMap<>();
    private static boolean initialized = false;

    /**
     * This method sets up this class and caches some commonly used bezier curves, such as the quadratic and cubic beziers.
     */
    public void initialize(){
        if (!initialized) {
            getCharacteristicMatrix(2); // quadratic bezier
            getCharacteristicMatrix(3); // cubic bezier
            getCharacteristicMatrix(4); // quartic bezier
            getCharacteristicMatrix(5); // quintic bezier
            initialized = true;
        }
    }

    /**
     * This method is to generate Pascal's triangle (all layers being left-aligned in a matrix) without the need for factorials
     * @param layers how many layers of the triangle to generate; minimum should be 1
     * @return Pascal's triangle (left-aligned)
     */
    private static double[][] generatePascalTriangle(int layers){
        double[][] output = new double[layers][layers];

        // pad the start and end of all layers with 1's
        for (int i = 0; i < layers; i++) {
            output[i][0] = 1;
            output[i][i] = 1;
        }

        // values in-between are calculated how Pascal's triangle layers are generated
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
    public static Matrix generateCharacteristicMatrix(int degree){
        // get a square matrix that contains Pascal's triangle
        Matrix output = new Matrix(generatePascalTriangle(degree + 1));

        // since a bezier's characteristic matrix has rows multiplied by the n-th layer of pascal's triangle respectively
        // construct an elementary row operation matrix that does those things
        Matrix rowMultOp = new Matrix(output.getRows(), output.getColumns());
        double[] mult = output.get(output.getRows() - 1);

        for (int i = 0; i < rowMultOp.getRows(); i++) {
            rowMultOp.set(i, i, mult[i]);
        }

        // multiply to obtain a characteristic matrix that doesn't have the appropriate negative signs yet
        output.multiply(rowMultOp);

        // flip signs if necessary
        // similar to the signs of a cofactor matrix
        int startingSign = 1;
        for (int i = 0; i < output.getRows(); i++) {

            double[] cachedRow = output.get(i);

            int realSign = startingSign;
            for (int j = 0; j < cachedRow.length; j++) {
                cachedRow[j] *= realSign;
                realSign *= -1;
            }
            output.set(i, cachedRow);
            startingSign *= -1;
        }

        return new Matrix(output);
    }

    /**
     * This method gets a characteristic matrix that is stored. If it doesn't exist, generate and return it.
     * @param degree bezier curve's degree
     * @return characteristic matrix of the Matrix class
     */
    public static Matrix getCharacteristicMatrix(int degree){
        if (!matrices.containsKey(degree)){
            matrices.put(degree, generateCharacteristicMatrix(degree));
        }
        return matrices.get(degree);
    }
}
