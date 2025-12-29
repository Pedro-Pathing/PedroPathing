package com.pedropathing.math;

import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.Locale;
import java.util.Objects;

/**
 * The Matrix class is used to represent matrix objects. Has basic operations such as adding, subtracting, and multiplying (w/ scalars and another matrix),
 * but also includes an implementation of gaussian elimination with partial pivoting and row operations
 *
 * @author William Phomphakdee - 7462 Not to Scale Alumni
 * @author Havish Sripada - 12808 RevAmped Robotics
 * @version 1.0.0, 08/29/2025
 */
public class Matrix {

    /**
     * internal array representation
     */
    private double[][] matrix;

    /**
     * cached row count because dims are immutable
     */
    private int rowCount;

    /**
     * cached column count because are immutable
     */
    private int colCount;

    /**
     * A constructor that creates a zero matrix of size 0x0
     * This is used for empty matrices
     */
    public Matrix() {
        this(0, 0);
    }

    /**
     * A constructor that creates a zero matrix of specified dimensions
     * @param rowCount number of rows
     * @param colCount number of columns
     */
    public Matrix(int rowCount, int colCount){
        this.rowCount = rowCount;
        this.colCount = colCount;
        this.matrix = new double[this.rowCount][this.colCount];

    }

    /**
     * A constructor that creates a matrix from a given 2d array
     * @param m 2d double array
     */
    public Matrix(double[][] m){
        setMatrix(m);
    }

    /**
     * This creates a new Matrix from another Matrix.
     *
     * @param setMatrix the Matrix input.
     */
    public Matrix(Matrix setMatrix) {
        setMatrix(setMatrix);
    }

    /**
     * This sets the 2D Array of this Matrix to a copy of the 2D Array of another Matrix.
     *
     * @param setMatrix the Matrix to copy from
     */
    public void setMatrix(Matrix setMatrix) {
        setMatrix(setMatrix.getMatrix());
    }

    /**
     * This sets the 2D Array of this Matrix to a copy of a specified 2D Array.
     *
     * @param setMatrix the 2D Array to copy from
     */
    public void setMatrix(double[][] setMatrix) {
        int columns = setMatrix[0].length;
        for (int i = 0; i < setMatrix.length; i++) {
            if (setMatrix[i].length != columns) {
                throw new IllegalArgumentException("matrices must be rectangular");
            }
        }
        matrix = deepCopy(setMatrix);
        rowCount = setMatrix.length;
        colCount = setMatrix[0].length;
    }

    /**
     * A method to return a separate object instance of the current matrix
     * @return a direct copy of the matrix
     */
    public Matrix copy(){
        Matrix output = new Matrix(this.rowCount, this.colCount);
        for (int i = 0; i < this.rowCount; i++) {
            double[] temp;
            temp = Arrays.copyOf(this.getRow(i), this.colCount);
            output.setRow(i, temp);
        }
        return output;
    }

    /**
     * This creates a copy of a 2D Array of doubles that references entirely new memory locations
     * from the original 2D Array of doubles, so no issues with mutability.
     *
     * @param copyMatrix the 2D Array of doubles to copy
     * @return returns a deep copy of the input Array
     */
    public static double[][] deepCopy(double[][] copyMatrix) {
        double[][] returnMatrix = new double[copyMatrix.length][copyMatrix[0].length];
        for (int i = 0; i < copyMatrix.length; i++) {
            returnMatrix[i] = Arrays.copyOf(copyMatrix[i], copyMatrix[i].length);
        }
        return returnMatrix;
    }

    /**
     * Get the internal representation of Matrix
     * @return 2d double array
     */
    public double[][] getMatrix() {
        return this.matrix;
    }

    /**
     * Get the number of rows this matrix has
     * @return number of rows
     */
    public int getRows() {
        return this.rowCount;
    }

    /**
     * Get the number of columns this matrix has
     * @return number of columns
     */
    public int getColumns() {
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
     * This returns a specified row of the Matrix in the form of an Array of doubles.
     *
     * @param row the index of the row to return
     * @return returns the row of the Matrix specified
     */
    public double[] get(int row) {
        return Arrays.copyOf(matrix[row], matrix[row].length);
    }

    /**
     * This sets a row of the Matrix to a copy of a specified Array of doubles.
     *
     * @param row the row to be written over
     * @param input the Array input
     * @return returns if the operation was successful
     */
    public boolean set(int row, double[] input) {
        if (input.length != getColumns()) {
            return false;
        }
        matrix[row] = Arrays.copyOf(input, input.length);
        return true;
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
        int len = Math.min(elements.length, this.colCount);
        if (len >= 0) System.arraycopy(elements, 0, this.matrix[row], 0, len);
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
    public Matrix plus(Matrix other){
        if (other.rowCount != this.rowCount || other.colCount != this.colCount)
            throw new RuntimeException(String.format("Size is not equal. size(A) = [%d, %d]; size(B) = [%d, %d]", this.rowCount, this.colCount, other.rowCount, other.colCount));

        Matrix output = new Matrix(this.rowCount, this.colCount);
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
    public Matrix minus(Matrix other){
        if (other.rowCount != this.rowCount || other.colCount != this.colCount)
            throw new RuntimeException(String.format("Size is not equal. size(A) = [%d, %d]; size(B) = [%d, %d]", this.rowCount, this.colCount, other.rowCount, other.colCount));

        Matrix output = new Matrix(this.rowCount, this.colCount);
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
    public Matrix multiply(double scalar){
        Matrix output = new Matrix(this.rowCount, this.colCount);
        if (scalar == 0) return Matrix.zeros(this.rowCount, this.colCount);

        for (int i = 0; i < output.rowCount; i++) {
            for (int j = 0; j < output.colCount; j++) {
                output.matrix[i][j] = scalar * this.matrix[i][j];
            }
        }
        return output;
    }

    /**
     * Returns a new matrix that has all elements multiplied by a scalar; cA
     * @param scalar a coefficient
     * @return a matrix with each element multiplied by the scalar
     */
    public Matrix times(double scalar){
        return this.multiply(scalar);
    }

    /**
     * Matrix multiplication between two matrices; A * B
     * @param other the other matrix (on the right side of the equation)
     * @return a new matrix that is the product of the two matrices
     */
    public Matrix multiply(Matrix other){
        if (other.rowCount != this.colCount)
            throw new IllegalArgumentException(String.format("Size mismatch for matrix multiplication. size(A) = [%d, %d]; size(B) = [%d, %d]", this.rowCount, this.colCount, other.rowCount, other.colCount));

        Matrix output = new Matrix(this.rowCount, other.colCount);

        for (int i = 0; i < output.rowCount; i++) {
            double[] rowSample = this.getRow(i);
            for (int j = 0; j < output.colCount; j++) {

                double dpSum = 0;
                double[] colSample = other.getCol(j);

                for (int k = 0; k < rowSample.length; k++) {
                    if (!(rowSample[k] == 0|| colSample[k] == 0)) {
                        dpSum += rowSample[k] * colSample[k];
                    }
                }

                output.matrix[i][j] = dpSum;

            }
        }

        return output;
    }

    /**
     * Matrix multiplication between two matrices; A * B
     * @param other the other matrix (on the right side of the equation)
     * @return a new matrix that is the product of the two matrices
     */
    public Matrix times(Matrix other){
        return this.multiply(other);
    }

    /**
     * Matrix multiplication between two matrices; A * B (remember, order matters here)
     * @param one the first matrix
     * @param two the second matrix
     * @return the product of the matrices
     */
    public static Matrix multiply(Matrix one, Matrix two) {
        return one.multiply(two);
    }

    /**
     * Returns a matrix that has all signs per element flipped
     * @return Additive inverse of the matrix
     */
    public Matrix unaryMinus(){
        return this.multiply(-1);
    }

    /**
     * Returns a new matrix that is the transpose of this matrix
     * @return transpose of this matrix; A^T
     */
    public Matrix transposed(){
        Matrix output = new Matrix(this.colCount, this.rowCount);
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
    public static Matrix[] rref(Matrix matrix, Matrix augment) {
        Matrix A = matrix.copy();
        Matrix B = augment.copy();

        int rowLim = A.getRows();
        int colLim = A.getColumns();
        int leadCol = 0;

        // Forward elimination
        for (int r = 0; r < rowLim && leadCol < colLim; r++) {

            int pivot = isPivotInCol(A, r, leadCol);
            while (pivot == -1) {
                leadCol++;
                if (leadCol >= colLim) {
                    return new Matrix[]{A, B};
                }
                pivot = isPivotInCol(A, r, leadCol);
            }

            A.rowSwap(r, pivot);
            B.rowSwap(r, pivot);

            for (int r2 = r + 1; r2 < rowLim; r2++) {
                if (A.get(r2, leadCol) != 0.0) {
                    double scalar = -A.get(r2, leadCol) / A.get(r, leadCol);
                    A.rowAdd(r, r2, scalar);
                    B.rowAdd(r, r2, scalar);
                }
            }

            leadCol++;
        }

        // Normalize pivot rows
        for (int r = 0; r < rowLim; r++) {
            for (int c = 0; c < colLim; c++) {
                if (A.get(r, c) != 0.0) {
                    double inv = 1.0 / A.get(r, c);
                    A.rowScale(r, inv);
                    B.rowScale(r, inv);
                    break;
                }
            }
        }

        // Back substitution (FIXED LOOP BOUND)
        for (int r = rowLim - 1; r >= 0; r--) {
            int pivotCol = -1;
            for (int c = 0; c < colLim; c++) {
                if (A.get(r, c) != 0.0) {
                    pivotCol = c;
                    break;
                }
            }

            if (pivotCol != -1) {
                for (int r2 = 0; r2 < r; r2++) {
                    double scalar = -A.get(r2, pivotCol);
                    A.rowAdd(r, r2, scalar);
                    B.rowAdd(r, r2, scalar);
                }
            }
        }

        return new Matrix[]{A, B};
    }


    /**
     * A helper method that investigates a matrix for a pivot on a specified column starting from a row
     * @param queriedMatrix a matrix the row operations are based on
     * @param startRow start row
     * @param col column where the pivot should be
     * @return row number of the pivot (-1 if no row is found)
     */
    private static int isPivotInCol(Matrix queriedMatrix, int startRow, int col){
        int best = -1;
        double max = 0.0;

        for (int r = startRow; r < queriedMatrix.getRows(); r++) {
            double v = Math.abs(queriedMatrix.get(r, col));
            if (v > max) {
                max = v;
                best = r;
            }
        }
        return max == 0.0 ? -1 : best;
    }

    /**
     * Computes the determinant of the matrix using Laplace Expansion
     * @precondition The matrix must be a square matrix
     * @return the determinant of the matrix
     */
    public double determinant() {
        if (rowCount != colCount)
            throw new IllegalStateException("Determinant only defined for square matrices");

        if (rowCount == 0) return 1;
        if (rowCount == 1) return matrix[0][0];
        if (rowCount == 2) {
            return matrix[0][0] * matrix[1][1]
                    - matrix[0][1] * matrix[1][0];
        }

        double det = 0;
        for (int j = 0; j < colCount; j++) {
            det += ((j % 2 == 0) ? 1 : -1)
                    * matrix[0][j]
                    * minor(0, j).determinant();
        }
        return det;
    }

    /**
     * Returns the minor of this matrix formed by removing the specified row
     * and column.
     *
     * @param skipRow the row to exclude
     * @param skipCol the column to exclude
     * @return the submatrix with the given row and column removed
     */
    private Matrix minor(int skipRow, int skipCol) {
        Matrix m = new Matrix(rowCount - 1, colCount - 1);
        int r = 0;

        for (int i = 0; i < rowCount; i++) {
            if (i == skipRow) continue;
            int c = 0;
            for (int j = 0; j < colCount; j++) {
                if (j == skipCol) continue;
                m.matrix[r][c++] = matrix[i][j];
            }
            r++;
        }
        return m;
    }

    /**
     * Computes the adjoint (adjugate) of this matrix
     * @precondition The matrix must be a square matrix
     * @return the adjoint of this matrix
     */
    public Matrix adjoint() {
        if (rowCount != colCount)
            throw new IllegalStateException("Adjoint only defined for square matrices");

        Matrix cofactors = new Matrix(rowCount, colCount);

        for (int i = 0; i < rowCount; i++) {
            for (int j = 0; j < colCount; j++) {
                double sign = ((i + j) % 2 == 0) ? 1 : -1;
                cofactors.matrix[i][j] = sign * minor(i, j).determinant();
            }
        }

        return cofactors.transposed();
    }

    /**
     * Gets the inverse matrix
     * @param matrix An invertible 2x2 matrix to compute the inverse of
     * @return the inverse of the given matrix
     */
    public static Matrix inverse2x2(Matrix matrix) {
        if (matrix.rowCount != 2 || matrix.colCount != 2)
            throw new IllegalStateException("Matrix is not 2x2");

        double det = matrix.determinant();
        if (det == 0.0)
            throw new ArithmeticException("Matrix is singular");

        Matrix inv = new Matrix(2, 2);
        inv.matrix[0][0] =  matrix.get(1,1) / det;
        inv.matrix[0][1] = -matrix.get(0,1) / det;
        inv.matrix[1][0] = -matrix.get(1,0) / det;
        inv.matrix[1][1] =  matrix.get(0,0) / det;

        return inv;
    }

    /**
     * Gets the inverse matrix
     * @param matrix An invertible 2x2 matrix to compute the inverse of
     * @return the inverse of the given matrix
     */
    public static Matrix inverse3x3(Matrix matrix) {
        if (matrix.rowCount != 3 || matrix.colCount != 3)
            throw new IllegalStateException("Matrix is not 3x3");

        double det = matrix.determinant();
        if (det == 0.0)
            throw new ArithmeticException("Matrix is singular");

        Matrix inv = new Matrix(3, 3);

        inv.matrix[0][0] =  (matrix.get(1,1)*matrix.get(2,2) - matrix.get(1,2)*matrix.get(2,1)) / det;
        inv.matrix[0][1] = -(matrix.get(0,1)*matrix.get(2,2) - matrix.get(0,2)*matrix.get(2,1)) / det;
        inv.matrix[0][2] =  (matrix.get(0,1)*matrix.get(1,2) - matrix.get(0,2)*matrix.get(1,1)) / det;

        inv.matrix[1][0] = -(matrix.get(1,0)*matrix.get(2,2) - matrix.get(1,2)*matrix.get(2,0)) / det;
        inv.matrix[1][1] =  (matrix.get(0,0)*matrix.get(2,2) - matrix.get(0,2)*matrix.get(2,0)) / det;
        inv.matrix[1][2] = -(matrix.get(0,0)*matrix.get(1,2) - matrix.get(0,2)*matrix.get(1,0)) / det;

        inv.matrix[2][0] =  (matrix.get(1,0)*matrix.get(2,1) - matrix.get(1,1)*matrix.get(2,0)) / det;
        inv.matrix[2][1] = -(matrix.get(0,0)*matrix.get(2,1) - matrix.get(0,1)*matrix.get(2,0)) / det;
        inv.matrix[2][2] =  (matrix.get(0,0)*matrix.get(1,1) - matrix.get(0,1)*matrix.get(1,0)) / det;

        return inv;
    }

    /**
     * Computes the inverse matrix using row reduction
     * @return the inverse matrix
     */
    public Matrix inverse() {
        if (rowCount != colCount)
            throw new IllegalStateException("Matrix must be square");

        Matrix I = Matrix.identity(rowCount);
        Matrix[] r = Matrix.rref(this, I);

        if (!r[1].equals(I)) throw new IllegalArgumentException("matrix not invertible");
        return r[1];
    }

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

    /**
     * Checks whether this matrix is approximately equal to another matrix.
     * Equality is determined element-wise within a given tolerance.
     *
     * @param other the matrix to compare against
     * @param eps numerical tolerance
     * @return true if matrices are equal within tolerance
     */
    public boolean equals(Matrix other, double eps) {
        if (other == null) return false;
        if (this.rowCount != other.rowCount || this.colCount != other.colCount)
            return false;

        for (int i = 0; i < rowCount; i++) {
            for (int j = 0; j < colCount; j++) {
                if (Math.abs(this.matrix[i][j] - other.matrix[i][j]) > eps)
                    return false;
            }
        }
        return true;
    }

    @Override
    public boolean equals(Object other) {
        if (!(other instanceof Matrix)) return false;
        return equals((Matrix) other, 1e-9);
    }

    @Override
    public int hashCode() {
        return Arrays.deepHashCode(getMatrix());
    }
}