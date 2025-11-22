package com.pedropathing.geometry;

import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;

import java.util.ArrayList;

public interface Curve {
    /**
     * This method initializes the curve. It should be called before any other methods are called.
     */
    void initialize();

    /**
     * This method returns the normal vector of the curve at a given parameter t.
     * @param t the parametric t value of the curve, which is to be clamped between 0 and 1 inclusive.
     * @return the normal vector at the given parameter t.
     */
    Vector getNormalVector(double t);

    /**
     * This method returns the reversed version of the curve.
     * @return the reversed curve.
     */
    Curve getReversed();

    /**
     * This method returns the derivative of the curve at a given parameter t.
     * @param t the parametric t value of the curve, which is to be clamped between 0 and 1 inclusive.
     * @return the derivative vector at the given parameter t.
     */
    Vector getDerivative(double t);

    /**
     * This method returns the pose of the curve at a given parameter t.
     * @param t the parametric t value of the curve, which is to be clamped between 0 and 1 inclusive.
     * @return the pose at the given parameter t.
     */
    Pose getPose(double t);

    /**
     * This method returns the second derivative of the curve at a given parameter t.
     * @param t the parametric t value of the curve, which is to be clamped between 0 and 1 inclusive.
     * @return the second derivative vector at the given parameter t.
     */
    Vector getSecondDerivative(double t);

    /**
     * This method checks if the curve is at the parametric end at a given parameter t.
     * @param t the parametric t value of the curve, which is to be clamped between 0 and 1 inclusive.
     * @return true if the curve is at the parametric end, false otherwise.
     */
    default boolean atParametricEnd(double t) {
        t = MathFunctions.clamp(t, 0, 1);
        return t >= getPathConstraints().getTValueConstraint();
    }

    /**
     * This method returns the ArrayList of control points for this curve.
     * @return the ArrayList of control points.
     */
    ArrayList<Pose> getControlPoints();

    /**
     * This method sets the path constraints for the curve.
     * @param constraints the PathConstraints to be set for the curve.
     */
    void setPathConstraints(PathConstraints constraints);

    /**
     * This method returns the path constraints for the curve.
     * @return the PathConstraints for the curve.
     */
    PathConstraints getPathConstraints();

    /**
     * This method returns the path completion at a given parameter t.
     * @param t the parametric t value of the curve, which is to be clamped between 0 and 1 inclusive.
     * @return the path completion at the given parameter t.
     */
    double getPathCompletion(double t);

    /**
     * This method returns the t value corresponding to a given path completion.
     * @param pathCompletion the path completion value, which is to be clamped between 0 and 1 inclusive.
     * @return the t value corresponding to the given path completion.
     */
    double getT(double pathCompletion);

    /**
     * This method returns the first control point of the curve.
     * @return the first control point.
     */
    default Pose getFirstControlPoint() {
        return getControlPoints().get(0);
    }

    /**
     * This method returns the last control point of the curve.
     * @return the last control point.
     */
    default Pose getLastControlPoint() {
        ArrayList<Pose> controlPoints = getControlPoints();
        return controlPoints.get(controlPoints.size() - 1);
    }

    /**
     * This method returns the second to last control point of the curve.
     * @return the second to last control point.
     */
    default Pose getSecondToLastControlPoint() {
        ArrayList<Pose> controlPoints = getControlPoints();
        return controlPoints.get(controlPoints.size() - 2);
    }

    /**
     * This method returns the second control point of the curve.
     * @return the second control point.
     */
    default Pose getSecondControlPoint() {
        ArrayList<Pose> controlPoints = getControlPoints();
        return controlPoints.get(1);
    }

    /**
     * This method returns the closest point on the curve to a given pose.
     * @param pose the pose to find the closest point to.
     * @param searchLimit the number of iterations to search for the closest point.
     * @param initialTValueGuess the initial guess for the t value of the closest point.
     * @return the t value of the closest point on the curve to the given pose.
     */
    default double getClosestPoint(Pose pose, int searchLimit, double initialTValueGuess) {
        for (int i = 0; i < searchLimit; i++) {
            Pose lastPoint = getPose(initialTValueGuess);

            Vector differenceVector = new Vector(lastPoint.minus(pose));

            double firstDerivative = 2 * getDerivative(initialTValueGuess).dot(differenceVector);
            double secondDerivative = 2 * (Math.pow(getDerivative(initialTValueGuess).getMagnitude(), 2) +
                    differenceVector.dot(getSecondDerivative(initialTValueGuess)));

            initialTValueGuess = MathFunctions.clamp(initialTValueGuess - firstDerivative / (secondDerivative + 1e-9), 0, 1);
            if (getPose(initialTValueGuess).distanceFrom(lastPoint) < 0.1) break;
        }

        return initialTValueGuess;
    }

    default double getClosestPoint(Pose pose, double initialTValueGuess) {
        return getClosestPoint(pose, getPathConstraints().getBEZIER_CURVE_SEARCH_LIMIT(), initialTValueGuess);
    }

    /**
     * This method returns the curvature of the curve at a given parameter t.
     * @param t the parametric t value of the curve, which is to be clamped between 0 and 1 inclusive.
     * @return the curvature at the given parameter t.
     */
    default double getCurvature(double t) {
        t = MathFunctions.clamp(t, 0, 1);

        Vector derivative = getDerivative(t);
        Vector secondDerivative = getSecondDerivative(t);

        if (derivative.getMagnitude() == 0) return 0;
        return (derivative.cross(secondDerivative))/Math.pow(derivative.getMagnitude(),3);
    }

    /**
     * This method returns the approximate length of the curve by sampling it at 1000 points.
     * @return the approximate length of the curve.
     */
    default double length() {
        Pose previousPoint = getPose(0);
        Pose currentPoint;
        double approxLength = 0;
        for (int i = 1; i <= 1000; i++) {
            currentPoint = getPose(i / 1000.0);
            approxLength += previousPoint.distanceFrom(currentPoint);
            previousPoint = currentPoint;
        }
        return approxLength;
    }

    /**
     * This method returns the approximate length of the curve by sampling it at 100 points.
     * @return the approximate length of the curve.
     */
    default double[][] getPanelsDrawingPoints() {
        double[][] panelsDrawingPoints = new double[2][101];
        for (int i = 0; i <= 100; i++) {
            Pose currentPoint = this.getPose(i/(double) (100));
            panelsDrawingPoints[0][i] = currentPoint.getX();
            panelsDrawingPoints[1][i] = currentPoint.getY();
        }

        return panelsDrawingPoints;
    }

    default Vector getEndTangent() {
        return getDerivative(1.0);
    }

    default String pathType() {
        return "other";
    }
}
