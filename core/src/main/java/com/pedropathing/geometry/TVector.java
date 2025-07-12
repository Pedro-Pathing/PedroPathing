package com.pedropathing.geometry;


import com.pedropathing.math.Matrix;

import java.util.Arrays;

/**
 * The TVector class represents the row vector matrix comprised of a polynomial for the matrix representation of a spline. This is NOT the Vector class.
 * <br>Eq: [1, t, t^2, t^3...] * P * C; where P is the characteristic matrix of the spline/curve while C is the control parameters
 *
 * @author icaras84
 * @version 0.0.1, 07/11/2025
 */
public class TVector {

    private int[][] diffPowers;
    private int[][] diffCoefficients;

    private int controlPointCount;
    private int differentiationLevel;

    /**
     * Construct a ParametricRowVector using two numbers: the number of control points a spline has, and the number of differentiations (including the 0th diff)
     * @param controlPointCount the number of control points a spline has
     * @param differentiationLevel the number of differentiations that can be requested (positional, velocity, acceleration, jerk... etc.)
     */
    public TVector(int controlPointCount, int differentiationLevel) {
        this.controlPointCount = controlPointCount;
        this.differentiationLevel = differentiationLevel;
        this.reload();
    }

    /**
     * Constructs a ParametricRowVector with the number of control points a spline has. The default differentiation level (including the 0th diff) is 3.
     * @param controlPointCount number of control points a spline has
     */
    public TVector(int controlPointCount) {
        this(controlPointCount, 3);
    }

    /**
     * Recompute what should be cached
     */
    public void reload(){
        initializeDegreeArray();
        initializeCoefficientArray();
    }

    /**
     * Initializes the degree/power array (for later processing) and cache them
     */
    private void initializeDegreeArray(){
        int deg = this.controlPointCount - 1;
        // for now, cache position, velocity, and acceleration powers (thus 3) per bezier obj (change to global caching later)
        this.diffPowers = new int[this.differentiationLevel][this.controlPointCount];

        for (int i = 0; i < this.diffPowers.length; i++) {
            this.diffPowers[i] = TVector.genDiff(deg, i);
        }
    }

    /**
     * Generate and return a polynomial's powers at the differentiation level
     * @param deg degree of poly
     * @param diffLevel number of differentiations
     * @return powers of each term in integers
     */
    private static int[] genDiff(int deg, int diffLevel){
        int[] output = new int[deg + 1];

        for (int i = diffLevel; i < output.length; i++) {
            output[i] = i - diffLevel;
        }

        return output;
    }

    /**
     * Initializes the coefficient array (for later processing) and cache them.
     * Each row is a different level of differentiation.
     */
    public void initializeCoefficientArray(){
        this.diffCoefficients = new int[this.differentiationLevel][this.controlPointCount];

        Arrays.fill(this.diffCoefficients[0], 1);

        for (int row = 1; row < this.diffCoefficients.length; row++) {
            for (int col = 0; col < this.diffCoefficients[0].length; col++) {
                this.diffCoefficients[row][col] = this.diffCoefficients[row - 1][col] * this.diffPowers[row - 1][col];
            }
        }
    }

    /**
     * This method gets the t-vector at the specified differentiation level.
     * @param t t value of the parametric curve; [0, 1]
     * @param diffLevel specifies how many differentiations are done
     * @return t vector
     */
    private double[] getTArray(double t, int diffLevel){
        int[] degrees = this.diffPowers[diffLevel];
        double[] powers = new double[this.controlPointCount];

        powers[0] = 1;
        for (int i = 1; i < powers.length; i++) {
            powers[i] = t * powers[i - 1];
        }

        double[] output = new double[powers.length];

        for (int i = 0; i < degrees.length; i++) {
            output[i] = powers[degrees[i]] * this.diffCoefficients[diffLevel][i];
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
        double[][] output = new double[diffTo + 1][this.controlPointCount];
        for (int i = 0; i < output.length; i++) {
            output[i] = getTArray(t, i);
        }
        return new Matrix(output);
    }

    public int[][] getDiffPowers() {
        return diffPowers;
    }

    public int[][] getDiffCoefficients() {
        return diffCoefficients;
    }

    public int getControlPointCount() {
        return controlPointCount;
    }

    public void setControlPointCount(int controlPointCount) {
        this.controlPointCount = controlPointCount;
        this.reload();
    }

    public int getDifferentiationLevel() {
        return differentiationLevel;
    }

    public void setDifferentiationLevel(int differentiationLevel) {
        this.differentiationLevel = differentiationLevel;
        this.reload();
    }
}
