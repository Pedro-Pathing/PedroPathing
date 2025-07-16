package com.pedropathing.geometry;

import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.MatrixUtil;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;

import java.util.ArrayList;

/**
 * A class to represent a spline defined in the form of TQC = X where:
 * <br>'T' is a 1xN matrix that represents something like [1, t, t^2, t^3...]
 * <br>'Q' is an NxN matrix that represents the characteristic matrix of a spline
 * <br>'C' is a Nx2 matrix that represents the control parameters of a spline (ex: in a bezier curve, this matrix's row vectors are the control points)
 * <br>'X' is a 1x2 matrix that contains the x and y of a point on the spline
 */
public class CharacteristicMatrixSpline implements Curve{

    private TVector tVectorGenerator;
    private Matrix splineMatrix;
    private Matrix controlMatrix;
    private Matrix cachedMatrix;
    private PathConstraints pathConstraints;


    /**
     * A constructor for the matrix representation of a spline
     * @param splineMatrix restriction: NxN matrix; linearly independent
     * @param controlMatrix restriction: NxM matrix
     */
    public CharacteristicMatrixSpline(Matrix splineMatrix, Matrix controlMatrix) {
        this.splineMatrix = splineMatrix;
        this.controlMatrix = controlMatrix;
        this.cachedMatrix = this.splineMatrix.multiply(this.controlMatrix);
        this.tVectorGenerator = new TVector(this.controlMatrix.getRows(), 3);
    }


    /**
     * Approximate the second derivative of the curve at a specified t value
     * @param t t value
     * @return second derivative Vector with an approximated heading
     */
    @Override
    public Vector getNormalVector(double t) {
        double current = getDerivative(t).getTheta();
        double deltaCurrent = getDerivative(t + 0.0001).getTheta();

        return new Vector(1, deltaCurrent - current);
    }

    /**
     * Get the spline if the control parameters were "reversed"
     * @return reversed curve
     */
    @Override
    public Curve getReversed() {
        Matrix reversedControlMatrix = new Matrix(this.controlMatrix);
        for (int i = 0; i < reversedControlMatrix.getRows() / 2; i++) {
            reversedControlMatrix.rowSwap(i, reversedControlMatrix.getRows() - i - 1);
        }
        return new CharacteristicMatrixSpline(this.splineMatrix, reversedControlMatrix);
    }

    /**
     * Get a point on the curve specified by the t value
     * @param t t value
     * @return Pose, but the heading is empty / zero
     */
    @Override
    public Pose getPose(double t) {
        t = MathFunctions.clamp(t, 0, 1);

        Matrix outPos = this.tVectorGenerator.getTMatrix(t, 9).multiply(this.cachedMatrix);
        return new Pose(outPos.get(0, 0), outPos.get(0, 1));
    }

    /**
     * Get the derivative of a point on the curve at the specified t value
     * @param t t value
     * @return the first derivative in Vector form
     */
    @Override
    public Vector getDerivative(double t) {
        t = MathFunctions.clamp(t, 0, 1);

        Matrix outVel = this.tVectorGenerator.getTMatrix(t, 1).multiply(this.cachedMatrix);
        Vector output = new Vector();
        output.setOrthogonalComponents(outVel.get(0, 0), outVel.get(0, 1));
        return output;
    }

    /**
     * Get the second derivative of a point on the curve at the specified t value
     * @param t t value
     * @return the second derivative in the form of a Vector
     */
    @Override
    public Vector getSecondDerivative(double t) {
        t = MathFunctions.clamp(t, 0, 1);

        Matrix outAccel = this.tVectorGenerator.getTMatrix(t, 2).multiply(this.cachedMatrix);
        Vector output = new Vector();
        output.setOrthogonalComponents(outAccel.get(0, 0), outAccel.get(0, 1));
        return output;
    }

    /**
     * A method to test if a t value is considered at the end of the curve
     * @param t t value
     * @return boolean
     */
    @Override
    public boolean atParametricEnd(double t) {
        return t >= this.pathConstraints.getTValueConstraint();
    }

    /**
     * This method calculates the position, velocity, and acceleration and puts them into a matrix
     * as row vectors.
     * @param t t value of the parametric curve; [0, 1]
     * @return matrix with row vectors corresponding to position, velocity, and acceleration at the requested t value
     */
    public Matrix getPointCharacteristics(double t){
        t = MathFunctions.clamp(t, 0, 1);
        return tVectorGenerator.getTMatrix(t, 3).multiply(this.cachedMatrix);
    }

    /**
     * Get the spline's control variables in the form of bezier control points
     * @return ArrayList of Poses (with empty / zero heading)
     */
    @Override
    public ArrayList<Pose> getControlPoints() {
        ArrayList<Pose> output = new ArrayList<>();
        Matrix transformedMatrix = Matrix.rref(CharacteristicMatrixSupplier.getBezierCharacteristicMatrix(this.controlMatrix.getRows() - 1), MatrixUtil.eye(this.controlMatrix.getRows()))[1].multiply(this.splineMatrix);
        Matrix transformedControlPoints = transformedMatrix.multiply(this.controlMatrix);
        for (double[] row : transformedControlPoints.getMatrix()) {
            output.add(new Pose(row[0], row[1]));
        }
        return output;
    }

    /**
     * Setter method for PathConstraints of this specific curve
     * @param constraints path constraints
     */
    @Override
    public void setPathConstraints(PathConstraints constraints) {
        this.pathConstraints = constraints;
    }

    /**
     * Getter method for PathConstraints of this specific curve
     * @return PathConstraints
      */
    @Override
    public PathConstraints getPathConstraints() {
        return this.pathConstraints;
    }

    /**
     * Calculate and return the curvature of a point on the curve at the specified t value
     * @param t t value
     * @return curvature (0 means no curvature / flat)
     */
    @Override
    public double getCurvature(double t) {
        t = MathFunctions.clamp(t, 0, 1);

        // [x  , y  ]
        // [x' , y' ]
        // [x'', y'']
        Matrix pointAtT = getPointCharacteristics(t);
        double derivativeSqLen = pointAtT.get(1, 0) * pointAtT.get(1, 0) + pointAtT.get(1, 1) * pointAtT.get(1, 1);
        double derivativeLen = Math.sqrt(derivativeSqLen);


        if (derivativeSqLen == 0) return 0;
        // MatLab notation (it's 1-based indexing, so be careful):
        // det(pointAtT(2:3, 1:2)) / (sum(pointAtT(1, 1:2).^2).^3)
        return (pointAtT.get(1, 0) * pointAtT.get(2, 1) - pointAtT.get(1, 1) * pointAtT.get(2, 0)) / (derivativeLen * derivativeSqLen);
    }

    /**
     * Getter method for the characteristic matrix for this spline
     * @return Matrix (NxN)
     */
    public Matrix getSplineMatrix() {
        return splineMatrix;
    }

    /**
     * Setter method for the characteristic matrix for this spline.
     * Will throw an error if the rows do not match the control matrix,
     * thus, set this after setting / updating the control matrix
     * @param splineMatrix new characteristic matrix; same restrictions as the constructor
     */
    public void setSplineMatrix(Matrix splineMatrix) {
        if (splineMatrix.getRows() != this.controlMatrix.getRows())
            throw new RuntimeException("Spline matrix must have rows equal to its control matrix.");
        this.splineMatrix = splineMatrix;
        this.cachedMatrix = this.splineMatrix.multiply(this.controlMatrix);
    }

    /**
     * Getter method for the control matrix. It is a matrix where it represents the control parameters of the spline
     * @return Matrix
     */
    public Matrix getControlMatrix() {
        return controlMatrix;
    }

    /**
     * Setter method for the control matrix
     * @param controlMatrix new / updated control matrix
     */
    public void setControlMatrix(Matrix controlMatrix) {
        this.controlMatrix = controlMatrix;
        this.tVectorGenerator.setControlPointCount(this.controlMatrix.getRows());
        this.cachedMatrix = this.splineMatrix.multiply(this.controlMatrix);
    }
}
