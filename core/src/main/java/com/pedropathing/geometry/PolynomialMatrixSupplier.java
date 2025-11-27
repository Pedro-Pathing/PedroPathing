package com.pedropathing.geometry;


import com.pedropathing.math.Matrix;
import com.pedropathing.math.PolyDiffUtil;

import java.util.Arrays;

/**
 * The PolynomialMatrixSupplier class represents the row vector matrix comprised of a polynomial for the matrix representation of a spline. This is NOT the Vector class.
 * <br>Eq: [1, t, t^2, t^3...] * P * C; where P is the characteristic matrix of the spline/curve while C is the control parameters
 *
 * @author William Phomphakdee - 7462 Not to Scale Alumni
 * @version 1.0.0, 11/026/2025
 */
public class PolynomialMatrixSupplier {

    private double[] preFilledArray;

    private int controlPointCount;

    /**
     * Constructs a ParametricRowVector with the number of control points a spline has. The default differentiation level (including the 0th diff) is 3.
     * @param controlPointCount number of control points a spline has
     */
    public PolynomialMatrixSupplier(int controlPointCount) {
        this.resizePoly(controlPointCount);
    }

    /**
     * This method gets the t-vector at the specified differentiation level.
     * @param t t value of the parametric curve; [0, 1]
     * @param diffLevel specifies how many differentiations are done
     * @return t vector
     */
    private double[] getTArray(double t, int diffLevel){
        double[] unmappedOutput = PolyDiffUtil.polyValArr(PolyDiffUtil.getPolyCoefficients(this.preFilledArray, diffLevel), t);
        double[] output = new double[unmappedOutput.length];
        System.arraycopy(unmappedOutput, 0, output, diffLevel, unmappedOutput.length - diffLevel);
        return output;
    }

    /**
     * Generates a matrix where each row is a differentiation level
     * @param t t value
     * @param diffs the array of differentiation levels (e.g. {0, 1, 2} for zeroth, first, and second differentiations)
     * @return a matrix of t vectors
     */
    private double[][] getTArray(double t, int[] diffs){
        double[][] coeff = new double[diffs.length][];
        for (int i = 0; i < diffs.length; i++) {
            coeff[i] = PolyDiffUtil.getPolyCoefficients(this.preFilledArray, diffs[i]);
        }

        double[][] unmappedOutputs = new double[coeff.length][];
        for (int i = 0; i < coeff.length; i++) {
            unmappedOutputs[i] = PolyDiffUtil.polyValArr(coeff[i], t);
        }

        double[][] output = new double[unmappedOutputs.length][this.preFilledArray.length];
        for (int i = 0; i < output.length; i++) {
            System.arraycopy(unmappedOutputs[i], 0, output[i], diffs[i], unmappedOutputs[i].length - diffs[i]);
        }

        return output;
    }

    /**
     * This method gets the t-vector at the specified differentiation level.
     * @param t t value of the parametric curve; [0, 1]
     * @param diffLevel specifies how many differentiations are done
     * @return t vector
     */
    public Matrix getRowVector(double t, int diffLevel){
        return new Matrix(new double[][]{getTArray(t, diffLevel)});
    }

    /**
     * Generates a matrix where each row is a differentiation level
     * @param t t value
     * @param diffTo the number corresponds to first, second, third... etc. derivative
     * @return a matrix of t vectors
     */
    public Matrix getTMatrix(double t, int diffTo){
        int[] differentiations = new int[diffTo + 1];
        for (int i = 1; i < differentiations.length; i++) {
            differentiations[i] = i;
        }

        return getTMatrix(t, differentiations);
    }

    /**
     * Generates a matrix where each row is a differentiation level
     * @param t t value
     * @param differentiations the array of differentiation levels (e.g. {0, 1, 2} for zeroth, first, and second differentiations)
     * @return a matrix of t vectors
     */
    public Matrix getTMatrix(double t, int[] differentiations){
        return new Matrix(getTArray(t, differentiations));
    }

    /**
     * This method resizes the internal polynomial coefficient array based on the new number of control points the polynomial spline has
     * @param controlPointCount new amount of control points
     */
    public void resizePoly(int controlPointCount){
        this.controlPointCount = controlPointCount;
        this.preFilledArray = new double[this.controlPointCount];
        Arrays.fill(this.preFilledArray, 1);
    }

    public int getControlPointCount() {
        return controlPointCount;
    }
}
