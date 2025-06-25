package com.pedropathing.math;

import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.Locale;

/**
 * The BLAMatrix class is used to represent matrix objects. Has basic operations such as adding, subtracting, and multiplying (w/ scalars and another matrix),
 * but also includes an implementation of gaussian elimination with partial pivoting and row operations
 *
 * @author icaras84
 * @version 1.0.0, 06/24/2025
 */
public class BLAMatrix {

    /**
     * internal array representation
     */
    private final double[][] matrix;

    /**
     * cached row count because dims are immutable
     */
    private final int rowCount;

    /**
     * cached column count because are immutable
     */
    private final int colCount;

    /**
     * A constructor that creates a zero matrix of specified dimensions
     * @param rowCount number of rows
     * @param colCount number of columns
     */
    public BLAMatrix(int rowCount, int colCount){
        this.rowCount = rowCount;
        this.colCount = colCount;
        this.matrix = new double[this.rowCount][this.colCount];

    }

    /**
     * A constructor that creates a matrix from a given 2d array
     * @param m 2d double array
     */
    public BLAMatrix(double[][] m){
       this.rowCount = m.length;
       this.colCount = m[0].length;
       this.matrix = new double[this.rowCount][this.colCount];
       for (int i = 0; i < this.rowCount; i++) {
           this.setRow(i, m[i]);
       }
    }

    /**
     * A method to return a separate object instance of the current matrix
     * @return a direct copy of the matrix
     */
    public BLAMatrix copy(){
        BLAMatrix output = new BLAMatrix(this.rowCount, this.colCount);
        for (int i = 0; i < this.rowCount; i++) {
            double[] temp = new double[this.colCount];
            temp = Arrays.copyOf(this.getRow(i), this.colCount);
            output.setRow(i, temp);
        }
        return output;
    }

    /**
     * Get the internal representation of BLAMatrix
     * @return 2d double array
     */
    public double[][] getMatrix() {
        return this.matrix;
    }

    /**
     * Get the number of rows this matrix has
     * @return number of rows
     */
    public int getRowCount() {
        return this.rowCount;
    }

    /**
     * Get the number of columns this matrix has
     * @return number of columns
     */
    public int getColCount() {
        return this.colCount;
    }

    /**
     * Get an array of length 2 that has the dimensions of the matrix. First element is number of rows, second element is number of columns.
     * @return 1d integer array
     */
    public int[] getSize(){
        return new int[]{this.rowCount, this.colCount};
    }

    /**
     * Get the specified row
     * @param row specifies which row of the matrix to return
     * @return a copy of the array of elements at the selected row
     */
    public double[] getRow(int row){
        return Arrays.copyOf(this.matrix[row], this.colCount);
    }

    /**
     * Replaces a specified row with new elements (left to right)
     * @param row specified row of the matrix
     * @param elements vararg of elements (must have a length that does not exceed the column count)
     */
    public void setRow(int row, double... elements){
        int len = Math.min(elements.length, this.rowCount);
        for (int i = 0; i < len; i++) {
            this.matrix[row][i] = elements[i];
        }
    }

    /**
     * Gets the specified column
     * @param col specified column of the matrix
     * @return a copy of an array of elements that represent the specified column of that matrix
     */
    public double[] getCol(int col){
        double[] output = new double[this.rowCount];
        for (int i = 0; i < this.rowCount; i++) {
            output[i] = this.matrix[i][col];
        }
        return output;
    }

    /**
     * Replaces a specified column with new elements (top to bottom)
     * @param col specified column of the matrix
     * @param elements vararg of elements (must have a length that does not exceed the row count)
     */
    public void setCol(int col, double... elements){
        int len = Math.min(elements.length, this.rowCount);
        for (int i = 0; i < len; i++) {
            this.matrix[i][col] = elements[i];
        }
    }

    /**
     * Gets the element at the specified coordinates
     * @param row which row the element is on (0-based)
     * @param col which column the element is on (0-based)
     * @return the element at the specified row and column
     */
    public double get(int row, int col){
        return this.matrix[row][col];
    }

    /**
     * Sets the element at the specified coordinates to another value
     * @param row which row the element is on (0-based)
     * @param col which column the element is on (0-based)
     * @param value the new value that the element is going to be replaced with
     */
    public void set(int row, int col, double value){
        this.matrix[row][col] = value;
    }

    /**
     * Returns the sum of the two matrices; A + B
     * @param other the other matrix (on the right side of the equation)
     * @return a matrix with elements equivalent to the sum of each of the two's respective elements
     */
    public BLAMatrix plus(BLAMatrix other){
        if (other.rowCount != this.rowCount || other.colCount != this.colCount)
            throw new RuntimeException(String.format("Size is not equal. size(A) = [%d, %d]; size(B) = [%d, %d]", this.rowCount, this.colCount, other.rowCount, other.colCount));

        BLAMatrix output = new BLAMatrix(this.rowCount, this.colCount);
        double[] nRow = new double[output.colCount];
        for (int i = 0; i < output.rowCount; i++) {
            for (int j = 0; j < nRow.length; j++) {
                nRow[j] = this.matrix[i][j] + other.matrix[i][j];
            }
            output.setRow(i, nRow);
        }
        return output;
    }

    /**
     * Returns a matrix that is the difference of the two matrices; A - B
     * @param other the other matrix (on the right side of the equation)
     * @return a matrix with elements equivalent to the difference of each of the two's respective elements
     */
    public BLAMatrix minus(BLAMatrix other){
        if (other.rowCount != this.rowCount || other.colCount != this.colCount)
            throw new RuntimeException(String.format("Size is not equal. size(A) = [%d, %d]; size(B) = [%d, %d]", this.rowCount, this.colCount, other.rowCount, other.colCount));

        BLAMatrix output = new BLAMatrix(this.rowCount, this.colCount);
        double[] nRow = new double[output.colCount];
        for (int i = 0; i < output.rowCount; i++) {
            for (int j = 0; j < nRow.length; j++) {
                nRow[j] = this.matrix[i][j] - other.matrix[i][j];
            }
            output.setRow(i, nRow);
        }
        return output;
    }

    /**
     * Returns a new matrix that has all elements multiplied by a scalar; cA
     * @param scalar a coefficient
     * @return a matrix with each element multiplied by the scalar
     */
    public BLAMatrix times(double scalar){
        BLAMatrix output = new BLAMatrix(this.rowCount, this.colCount);
        for (int i = 0; i < output.rowCount; i++) {
            for (int j = 0; j < output.colCount; j++) {
                output.matrix[i][j] = scalar * this.matrix[i][j];
            }
        }
        return output;
    }

    /**
     * Matrix multiplication between two matrices; A * B
     * @param other the other matrix (on the right side of the equation)
     * @return a new matrix that is the product of the two matrices
     */
    public BLAMatrix times(BLAMatrix other){
        if (other.rowCount != this.colCount)
            throw new RuntimeException(String.format("Size mismatch for matrix multiplication. size(A) = [%d, %d]; size(B) = [%d, %d]", this.rowCount, this.colCount, other.rowCount, other.colCount));

        BLAMatrix output = new BLAMatrix(this.rowCount, other.colCount);

        for (int i = 0; i < output.rowCount; i++) {
            double[] rowSample = this.getRow(i);
            for (int j = 0; j < output.colCount; j++) {

                double dpSum = 0;
                double[] colSample = other.getCol(j);

                for (int k = 0; k < rowSample.length; k++) {
                    dpSum += rowSample[k] * colSample[k];
                }

                output.matrix[i][j] = dpSum;

            }
        }

        return output;
    }

    /**
     * Returns a matrix that has all signs per element flipped
     * @return Additive inverse of the matrix
     */
    public BLAMatrix unaryMinus(){
        return this.times(-1);
    }

    /**
     * Returns a new matrix that is the transpose of this matrix
     * @return transpose of this matrix; A^T
     */
    public BLAMatrix transposed(){
        BLAMatrix output = new BLAMatrix(this.colCount, this.rowCount);
        for (int i = 0; i < this.colCount; i++) {
            output.setRow(i, this.getCol(i));
        }
        return output;
    }

    /**
     * Swap two rows in the matrix
     * @param srcRow row to swap
     * @param destRow row to swap to
     */
    public void rowSwap(int srcRow, int destRow){
        double[] cachedRow = Arrays.copyOf(this.matrix[srcRow], this.colCount);
        this.matrix[srcRow] = Arrays.copyOf(this.matrix[destRow], this.colCount);
        this.matrix[destRow] = Arrays.copyOf(cachedRow, cachedRow.length);
    }

    /**
     * Multiplies a who row by a scalar in the matrix
     * @param row specified row to do operation
     * @param scalar a scalar to multiply all elements in that row by
     */
    public void rowScale(int row, double scalar){
        for (int i = 0; i < this.colCount; i++) {
            this.matrix[row][i] *= scalar;
        }
    }

    /**
     * Adds a row to another row that is scaled by a scalar
     * @param srcRow row to add
     * @param destRow row to add to
     * @param scalar a scalar that multiplies the elements to be added
     */
    public void rowAdd(int srcRow, int destRow, double scalar){
        for (int i = 0; i < this.colCount; i++) {
            this.matrix[destRow][i] += this.matrix[srcRow][i] * scalar;
        }
    }

    /**
     * Perform gaussian elimination on a matrix and its augment to achieve a row-reduced echelon form (hence RREF)
     * @param matrix a matrix of MxN size
     * @param augment an augment matrix that has the same number of rows as the matrix
     * @return a 1d array of two matrices --the first element is the matrix, the second element is the augment matrix; rref([A|B])
     */
    public static BLAMatrix[] rref(BLAMatrix matrix, BLAMatrix augment){
        BLAMatrix outputMatrix = matrix.copy();
        BLAMatrix outputAugment = augment.copy();

        int rowLim = outputMatrix.getRowCount();
        int colLim = outputMatrix.getColCount();
        int currentCol = 0;

        for (int r1 = 0; r1 < rowLim; r1++) {
            for (int c1 = 0; c1 < colLim; c1++) {
                int pivot = BLAMatrix.isPivotInCol(outputMatrix, r1, c1);
                if (pivot != -1){
                    currentCol = c1;
                    outputMatrix.rowSwap(r1, pivot);
                    outputAugment.rowSwap(r1, pivot);
                    break;
                }
            }

            for (int r2 = r1 + 1; r2 < rowLim; r2++) {
                if (outputMatrix.get(r2, currentCol) != 0.0){
                    double scalar = -outputMatrix.get(r2, currentCol) / outputMatrix.get(r1, currentCol);
                    outputMatrix.rowAdd(r1, r2, scalar);
                    outputAugment.rowAdd(r1, r2, scalar);
                }
            }
        }

        for (int r1 = 0; r1 < rowLim; r1++) {
            int pivotCol = -1;
            for (int c1 = 0; c1 < colLim; c1++) {
                if (outputMatrix.get(r1, c1) != 0.0){
                    pivotCol = c1;
                    break;
                }
            }

            if (pivotCol != -1) {
                double scalar = 1d / outputMatrix.get(r1, pivotCol);
                outputMatrix.rowScale(r1, scalar);
                outputAugment.rowScale(r1, scalar);
            }
        }

        for (int r1 = colLim - 1; r1 > 0; r1--) {
            int pivotCol = -1;
            for (int c1 = 0; c1 < colLim; c1++) {
                if (outputMatrix.get(r1, c1) != 0.0){
                    pivotCol = c1;
                    break;
                }
            }

            if (pivotCol != -1){
                for (int r2 = 0; r2 < r1; r2++) {
                    double scalar = -outputMatrix.get(r2, pivotCol);
                    outputMatrix.rowAdd(r1, r2, scalar);
                    outputAugment.rowAdd(r1, r2, scalar);
                }
            }
        }

        return new BLAMatrix[]{outputMatrix, outputAugment};
    }

    /**
     * A helper method that investigates a matrix for a pivot on a specified column starting from a row
     * @param queriedMatrix a matrix the row operations are based on
     * @param startRow start row
     * @param col column where the pivot should be
     * @return row number of the pivot (-1 if no row is found)
     */
    private static int isPivotInCol(BLAMatrix queriedMatrix, int startRow, int col){
        int output = -1;

        for (int r1 = startRow; r1 < queriedMatrix.getRowCount(); r1++) {
            if (queriedMatrix.get(r1, col) != 0.0){
                output = r1;
                break;
            }
        }

        return output;
    }

    /**
     * Build a string that represents the elements of the matrix
     * @return String obj
     */
    @NotNull
    @Override
    public String toString(){
        StringBuilder builder = new StringBuilder("[");
        for (int i = 0; i < this.rowCount; i++) {
            for (int j = 0; j < this.colCount; j++) {
                builder.append(String.format(Locale.getDefault(), "%.5f, ", this.matrix[i][j]));
            }
            builder.append("\b\b; ");
        }
        builder.append("\b\b]");
        return builder.toString();
    }
}
