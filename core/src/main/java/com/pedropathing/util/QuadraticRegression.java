package com.pedropathing.util;

import java.util.List;

/**
 * Finds the best-fit quadratic curve of the form y = ax^2 + bx.
 * Used for tuning predicted braking distances.
 *
 * @author Jacob Ophoven - 18535, Frozen Code
 */
class QuadraticRegression {
    public static double[] quadraticFit(List<double[]> points) {
        double sumX2 = 0, sumX3 = 0, sumX4 = 0;
        double sumXY = 0, sumX2Y = 0;
        
        for (double[] point : points) {
            double x = point[0];
            double y = point[1];
            
            sumX2 += x * x;
            sumX3 += x * x * x;
            sumX4 += x * x * x * x;
            sumXY += x * y;
            sumX2Y += x * x * y;
        }
        
        double[][] matrix = {
            {sumX2, sumX3},
            {sumX3, sumX4}
        };
        
        double[] constants = {sumXY, sumX2Y};
        
        return solveLinearSystem(matrix, constants); // returns {b, a}
    }

    private static double[] solveLinearSystem(double[][] A, double[] B) {
        int n = B.length;
        double[] X = new double[n];
        double detA = determinant(A);

        if (Math.abs(detA) < 1e-10) {
            throw new IllegalArgumentException("Matrix is singular or nearly singular");
        }

        for (int i = 0; i < n; i++) {
            double[][] Ai = replaceColumn(A, B, i);
            X[i] = determinant(Ai) / detA;
        }

        return X;
    }

    private static double determinant(double[][] matrix) {
        return matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
    }

    private static double[][] replaceColumn(double[][] matrix, double[] column, int colIndex) {
        double[][] newMatrix = new double[matrix.length][matrix[0].length];

        for (int i = 0; i < matrix.length; i++) {
            System.arraycopy(matrix[i], 0, newMatrix[i], 0, matrix[i].length);
            newMatrix[i][colIndex] = column[i];
        }

        return newMatrix;
    }
}
