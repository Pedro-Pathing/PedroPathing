package com.pedropathing.geometry;

import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;

import java.util.ArrayList;
import java.util.List;

/**
 * This class is a representation of a quintic hermite spline.
 * This will optimize a curve best fit for approach and exit velocity and accelerations, however,
 * this does not guarantee that the robot will be constrained by those velocity and acceleration variables.
 * Merely, these variables dictate the shape of the curve.
 *
 * @author icaras84
 * @version 0.0.1, 07/11/2025
 */
public class QuinticHermiteCurve implements Curve{

    /**
     * this is the characteristic matrix for a quintic hermite spline in matrix representation form
     * it does not need to be stored in every object of quintic hermite spline, thus the static modifier
     */
    private static final Matrix characteristicMatrix = CharacteristicMatrixSupplier.generateQuinticHermiteMatrix();;

    private Pose beginPose;

    private Vector beginVel;

    private Vector beginAccel;


    private Pose endPose;
    private Vector endVel;
    private Vector endAccel;

    private Matrix preMultControl;

    private final TVector tVector = new TVector(6, 4);

    private PathConstraints pathConstraints;


    /**
     * Quintic Hermite Spline constructor that accepts the following for controlling the shape:
     * @param beginPose position will the spline begin at
     * @param beginVel beginning curve velocity (does NOT translate to a velocity constraint)
     * @param beginAccel beginning curve acceleration (does NOT translate to an acceleration constraint)
     * @param endPose position will the spline end at
     * @param endVel ending curve velocity (does NOT translate to a velocity constraint)
     * @param endAccel ending curve acceleration (does NOT translate to an acceleration constraint)
     */
    public QuinticHermiteCurve(Pose beginPose, Vector beginVel, Vector beginAccel, Pose endPose, Vector endVel, Vector endAccel) {
        this.beginPose = beginPose;
        this.beginVel = beginVel;
        this.beginAccel = beginAccel;
        this.endPose = endPose;
        this.endVel = endVel;
        this.endAccel = endAccel;
        calculatePreMultControl();
    }

    private void calculatePreMultControl(){
        Matrix controls = new Matrix(new double[][]{
                {beginPose.getX(), beginPose.getY()},
                {beginVel.getXComponent(), beginVel.getYComponent()},
                {beginAccel.getXComponent(), beginAccel.getYComponent()},
                {endPose.getX(), endPose.getY()},
                {endVel.getXComponent(), endVel.getYComponent()},
                {endAccel.getXComponent(), endAccel.getYComponent()}
        });

        this.preMultControl = characteristicMatrix.multiply(controls);
    }

    @Override
    public Vector getNormalVector(double t) {
        double current = getDerivative(t).getTheta();
        double deltaCurrent = getDerivative(t + 0.0001).getTheta();

        return new Vector(1, deltaCurrent - current);
    }

    @Override
    public QuinticHermiteCurve getReversed() {
        return new QuinticHermiteCurve(this.endPose, this.endVel, this.endAccel, this.beginPose, this.beginVel, this.beginAccel);
    }

    @Override
    public Vector getDerivative(double t) {
        t = Math.clamp(t, 0, 1);
        Matrix tvec = tVector.getRowVector(t, 1);
        Matrix output = tvec.multiply(this.preMultControl);
        Vector out = new Vector();
        out.setOrthogonalComponents(output.get(0, 0), output.get(0, 1));
        return out;
    }

    @Override
    public Pose getPose(double t) {
        t = Math.clamp(t, 0, 1);
        Matrix tvec = tVector.getRowVector(t, 0);
        Matrix output = tvec.multiply(this.preMultControl);
        return new Pose(output.get(0, 0), output.get(0, 1));
    }

    @Override
    public Vector getSecondDerivative(double t) {
        t = Math.clamp(t, 0, 1);
        Matrix tvec = tVector.getRowVector(t, 2);
        Matrix output = tvec.multiply(this.preMultControl);
        Vector out = new Vector();
        out.setOrthogonalComponents(output.get(0, 0), output.get(0, 1));
        return out;
    }

    @Override
    public boolean atParametricEnd(double t) {
        return t >= pathConstraints.getTValueConstraint();
    }

    @Override
    public ArrayList<Pose> getControlPoints() {
        Pose control1 = getSecondControlPoint();
        Pose control2 = beginPose.plus(new Pose(beginVel.getXComponent(), beginVel.getYComponent()).times(0.4d))
                .plus(new Pose(beginAccel.getXComponent(), beginAccel.getYComponent()).times(0.05d));
        Pose control3 = endPose.minus(new Pose(beginVel.getXComponent(), beginVel.getYComponent()).times(0.4d))
                .plus(new Pose(endAccel.getXComponent(), endAccel.getYComponent()).times(0.05d));
        Pose control4 = getSecondToLastControlPoint();

        return new ArrayList<>(List.of(beginPose, control1, control2, control3, control4, endPose));
    }

    @Override
    public void setPathConstraints(PathConstraints constraints) {
        this.pathConstraints = constraints;
    }

    public PathConstraints getPathConstraints() {
        return pathConstraints;
    }

    @Override
    public Pose getFirstControlPoint() {
        return this.beginPose;
    }

    @Override
    public Pose getLastControlPoint() {
        return this.endPose;
    }

    @Override
    public Pose getSecondToLastControlPoint() {
        return endPose.minus(new Pose(beginVel.getXComponent(), beginVel.getYComponent()).times(0.2d));
    }

    @Override
    public Pose getSecondControlPoint() {
        return beginPose.plus(new Pose(beginVel.getXComponent(), beginVel.getYComponent()).times(0.2d));
    }

    /**
     * This returns the starting position of the curve
     * @return Pose
     */
    public Pose getBeginPose() {
        return beginPose;
    }

    /**
     * Set the beginning position and recompute the cached matrices.
     * @param beginPose beginning position of the curve (as a pose)
     */
    public void setBeginPose(Pose beginPose) {
        this.beginPose = beginPose;
        calculatePreMultControl();
    }

    /**
     * This returns the starting curve velocity (this is NOT a velocity constraint on the robot)
     * @return curve velocity / gradient
     */
    public Vector getBeginVel() {
        return beginVel;
    }

    /**
     * Set the curve velocity at the beginning of the curve and recompute cached matrices (this is NOT a velocity constraint on the robot)
     * @param beginVel curve velocity / gradient in the form of a Vector
     */
    public void setBeginVel(Vector beginVel) {
        this.beginVel = beginVel;
        calculatePreMultControl();
    }

    /**
     * This returns the starting curve acceleration (this is NOT an acceleration constraint on the robot)
     * @return curve acceleration / gradient of the gradient
     */
    public Vector getBeginAccel() {
        return beginAccel;
    }

    /**
     * Sets the curve acceleration at the beginning of the curve (this is NOT an acceleration constraint on the robot)
     * @param beginAccel curve acceleration in the form of a Vector
     */
    public void setBeginAccel(Vector beginAccel) {
        this.beginAccel = beginAccel;
        calculatePreMultControl();
    }

    /**
     * This returns the end position of the curve
     * @return Pose
     */
    public Pose getEndPose() {
        return endPose;
    }

    /**
     * Set the end position and recompute the cached matrices.
     * @param endPose end position of the curve (as a pose)
     */
    public void setEndPose(Pose endPose) {
        this.endPose = endPose;
        calculatePreMultControl();
    }

    /**
     * This returns the ending curve velocity (this is NOT a velocity constraint on the robot)
     * @return curve velocity / gradient
     */
    public Vector getEndVel() {
        return endVel;
    }

    /**
     * Set the curve velocity at the end of the curve and recompute cached matrices (this is NOT a velocity constraint on the robot)
     * @param endVel curve velocity / gradient in the form of a Vector
     */
    public void setEndVel(Vector endVel) {
        this.endVel = endVel;
        calculatePreMultControl();
    }

    /**
     * This returns the ending curve acceleration (this is NOT an acceleration constraint on the robot)
     * @return curve acceleration / gradient of the gradient
     */
    public Vector getEndAccel() {
        return endAccel;
    }

    /**
     * Sets the curve acceleration at the end of the curve (this is NOT an acceleration constraint on the robot)
     * @param endAccel curve acceleration in the form of a Vector
     */
    public void setEndAccel(Vector endAccel) {
        this.endAccel = endAccel;
        calculatePreMultControl();
    }
}
