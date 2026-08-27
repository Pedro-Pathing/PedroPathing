package com.pedropathing.paths.curves.bezier;

import com.pedropathing.math.Matrix;

import java.util.Arrays;

public class PolynomialMatrix {
    public static final int[][] COEFF_LUT = new int[4][100];
    static {
        int[] row1 = new int[COEFF_LUT[0].length];
        Arrays.fill(row1, 1);

        int[] row2 = new int[COEFF_LUT[1].length];
        for (int i = 0; i < row2.length; i++) {
            row2[i] = i;
        }

        COEFF_LUT[0] = row1;
        COEFF_LUT[1] = row2;

        for (int diffLevel = 2; diffLevel < COEFF_LUT.length; diffLevel++) {

            for (int i = diffLevel; i < COEFF_LUT[diffLevel].length; i++) {
                COEFF_LUT[diffLevel][i] = COEFF_LUT[diffLevel - 1][i] * COEFF_LUT[diffLevel - 1][i - 1];
            }
        }
    }

    private final int controlPointCount;

    public PolynomialMatrix(int controlPointCount) {
        this.controlPointCount = controlPointCount;
    }

    public int getControlPointCount() {
        return controlPointCount;
    }

    public Matrix getTMatrix(int diffLevel, double t) {
        int[] coeff = new int[this.controlPointCount];
        System.arraycopy(COEFF_LUT[diffLevel], 0, coeff, 0, this.controlPointCount);

        double[] output = new double[this.controlPointCount];
        double tInput = 1;

        for (int i = diffLevel; i < this.controlPointCount; i++) {
            output[i] = tInput * coeff[i];
            tInput *= t;
        }

        return new Matrix(new double[][]{output});
    }

    public Matrix getTMatrix(int diffLevel, double[] tValues) {
        int[] coeff = new int[this.controlPointCount];
        System.arraycopy(COEFF_LUT[diffLevel], 0, coeff, 0, this.controlPointCount);

        double[][] output = new double[tValues.length][this.controlPointCount];
        double[] tInput = new double[tValues.length];
        Arrays.fill(tInput, 1.0);

        for (int i = diffLevel; i < this.controlPointCount; i++) {
            for (int j = 0; j < tInput.length; j++) {
                output[j][i] = tInput[j] * coeff[i];
                tInput[j] *= tValues[j];
            }
        }

        return new Matrix(output);
    }

    public Matrix getTMatrix(int[] diffLevel, double t) {
        int[][] coeff = new int[diffLevel.length][this.controlPointCount];

        for (int i = 0; i < coeff.length; i++) {
            System.arraycopy(COEFF_LUT[diffLevel[i]], 0, coeff[i], 0, this.controlPointCount);
        }

        double[][] output = new double[diffLevel.length][this.controlPointCount];
        double[] tInput = new double[this.controlPointCount];

        tInput[0] = 1;
        for (int i = 1; i < tInput.length; i++) {
            tInput[i] = tInput[i - 1] * t;
        }

        for (int diffIdx = 0; diffIdx < coeff.length; diffIdx++) {
            for (int i = diffLevel[diffIdx]; i < this.controlPointCount; i++) {
                output[diffIdx][i] = coeff[diffIdx][i] * tInput[i - diffLevel[diffIdx]];
            }
        }

        return new Matrix(output);
    }
}