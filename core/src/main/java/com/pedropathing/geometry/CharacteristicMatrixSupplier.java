package com.pedropathing.geometry;

import com.pedropathing.math.Matrix;

import java.util.HashMap;

/**
 * The CharacteristicMatrixSupplier handles supply the characteristic matrices for splines in their matrix representations.
 *
 * @author William Phomphakdee - 7462 Not to Scale Alumni
 * @version 0.0.2, 08/29/2025
 */
public class CharacteristicMatrixSupplier {

    private static final HashMap<Integer, Matrix> bezierMatrices = new HashMap<>();
    private static boolean initialized = false;

    /**
     * This method sets up this class and caches some commonly used Bézier curves,
     * such as the quadratic and cubic Béziers.
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
     * This method generate Pascal's triangle with alternating signs (all values are left-aligned in the matrix)
     * @param layers how many layers of the triangle to generate; the minimum should be 1
     * @return Pascal's triangle (left-aligned)
     */
    private static double[][] generatePascalTriangle(int layers) {
        double[][] output = new double[layers][layers];

        // pad the start and end of all layers with 1's
        int prevSign = 1;
        for (int i = 0; i < layers; i++) {
            output[i][0] = prevSign;
            output[i][i] = 1;
            prevSign *= -1;
        }

        // apply the rule for the alternating sign pascal's triangle 'b - a = c' instead of the regular 'a + b = c'
        for (int i = 2; i < output.length; i++) {
            for (int j = 1; j < i; j++) {
                output[i][j] = output[i - 1][j - 1] - output[i - 1][j];
            }
        }

        return output;
    }

    /**
     * This method generates the characteristic matrix based on the degree of a requested Bézier curve.
     * 2 for quadratic, 3 for cubic, 4 for quartic, 5 for quintic... etc.
     * @param degree bezier curve's degree
     * @return characteristic matrix of the Matrix class
     */
    public static Matrix generateBezierCharacteristicMatrix(int degree){
        // get a square matrix that contains Pascal's triangle
        Matrix output = new Matrix(generatePascalTriangle(degree + 1));

        // sample the last row and multiply all other rows by the corresponding value
        double[] sampledRow = output.getRow(output.getRows() - 1);
        for (int i = 1; i < output.getRows() - 1; i++) {
            for (int j = 0; j <= i; j++) {
                // I wasn't sure if bitwise math to obtain an unsigned double is valid here since big endian vs little endian
                output.set(i, j, output.get(i, j) * Math.abs(sampledRow[i]));
            }
        }

        return output;
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
}