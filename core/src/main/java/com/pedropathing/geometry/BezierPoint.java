package com.pedropathing.geometry;

import com.pedropathing.math.Vector;

import java.util.ArrayList;

/**
 * This is the BezierPoint class. This class handles the creation of BezierPoints, which is what I
 * call Bezier curves with only one control point. The parent BezierCurve class cannot handle Bezier
 * curves with less than three control points, so this class handles points. Additionally, it makes
 * the calculations done on the fly a little less computationally expensive and more streamlined.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/9/2024
 */
public class BezierPoint extends BezierCurve {
    private Pose pose;
    private Vector endTangent = new Vector();
    private double length;

    /**
     * This creates a new BezierPoint with a specified Point.
     * This is just a point but it extends the BezierCurve class so things work.
     *
     * @param point the specified point.
     */


    /**
     * This creates a new BezierPoint with a specified Point.
     * This is just a point but it extends the BezierCurve class so things work.
     *
     * @param pose the specified point.
     */
    public BezierPoint(Pose pose) {
        super();
        this.pose = pose;
        length = approximateLength();
        super.initializePanelsDrawingPoints();
    }

    /**
     * This creates a new BezierPoint with a specified FuturePose.
     * This is just a point but it extends the BezierCurve class so things work.
     * If the FuturePose is not initialized, it will store the FuturePose for later initialization.
     *
     * @param pose the specified FuturePose.
     */
    public BezierPoint(FuturePose pose) {
        super();
        length = 0.0;
        if (pose.initialized()) {
            this.pose = pose.getPose();
            super.initializePanelsDrawingPoints();
        } else {
            futureControlPoints = new ArrayList<>();
            futureControlPoints.add(pose);
            lazyInitialize = true;
        }
    }

    /**
     * This creates a new BezierPoint with specified cartesian coordinates.
     * @param x the x-coordinate of the BezierPoint
     * @param y the y-coordinate of the BezierPoint
     */
    public BezierPoint(double x, double y) {
        this(new Pose(x, y));
    }

    /**
     * This supposedly returns the unit tangent Vector at the end of the path, but since there is
     * no end tangent of a point, this returns a zero Vector instead. Holding BezierPoints in the
     * Follower doesn't use the drive Vector, so the end tangent Vector is not needed or truly used.
     *
     * @return returns the zero Vector.
     */
    @Override
    public Vector getEndTangent() {
        return endTangent.copy();
    }

    /**
     * This gets the length of the BezierPoint. Since points don't have length, this returns zero.
     *
     * @return returns the length of the BezierPoint.
     */
    @Override
    public double approximateLength() {
        return 0.0;
    }

    /**
     * This returns the point on the BezierPoint that is specified by the parametric t value. Since
     * this is a Point, this just returns the one control point's position.
     *
     * @param t this is the t value of the parametric line. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the Point requested.
     */
    @Override
    public Pose getPose(double t) {
        if (pose == null) initialize();
        return new Pose(pose.getX(), pose.getY());
    }

    /**
     * This returns the curvature of the BezierPoint, which is zero since this is a Point.
     *
     * @param t the parametric t value.
     * @return returns the curvature, which is zero.
     */
    @Override
    public double getCurvature(double t) {
        return 0.0;
    }

    /**
     * This returns the derivative on the BezierPoint, which is the zero Vector since this is a Point.
     * The t value doesn't really do anything, but it's there so I can override methods.
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the derivative requested, which is the zero Vector.
     */
    @Override
    public Vector getDerivative(double t) {
        return endTangent.copy();
    }

    /**
     * This returns the second derivative on the Bezier line, which is the zero Vector since this
     * is a Point.
     * Once again, the t is only there for the override.
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the second derivative requested, which is the zero Vector.
     */
    @Override
    public Vector getSecondDerivative(double t) {
        return new Vector();
    }

    /**
     * This returns the zero Vector, but it's here so I can override the method in the BezierCurve
     * class.
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the approximated second derivative, which is the zero Vector.
     */
    @Override
    public Vector getNormalVector(double t) {
        return new Vector();
    }

    /**
     * Returns the ArrayList of control points for this BezierPoint
     *
     * @return This returns the control point.
     */
    @Override
    public ArrayList<Pose> getControlPoints() {
        ArrayList<Pose> returnList = new ArrayList<>();
        returnList.add(pose);
        return returnList;
    }

    /**
     * Returns the first, and only, control point for this BezierPoint
     *
     * @return This returns the Point.
     */
    @Override
    public Pose getFirstControlPoint() {
        return pose;
    }

    /**
     * Returns the second control point, or the one after the start, for this BezierPoint. This
     * returns the one control point of the BezierPoint, and there are several redundant methods
     * that return the same control point, but it's here so I can override methods.
     *
     * @return This returns the Point.
     */
    @Override
    public Pose getSecondControlPoint() {
        return pose;
    }

    /**
     * Returns the second to last control point for this BezierPoint. This
     * returns the one control point of the BezierPoint, and there are several redundant methods
     * that return the same control point, but it's here so I can override methods.
     *
     * @return This returns the Point.
     */
    @Override
    public Pose getSecondToLastControlPoint() {
        return pose;
    }

    /**
     * Returns the last control point for this BezierPoint. This
     * returns the one control point of the BezierPoint, and there are several redundant methods
     * that return the same control point, but it's here so I can override methods.
     *
     * @return This returns the Point.
     */
    @Override
    public Pose getLastControlPoint() {
        return pose;
    }

    /**
     * Returns the length of this BezierPoint, which is zero since Points don't have length.
     *
     * @return This returns the length.
     */
    @Override
    public double length() {
        return length;
    }

    /**
     * Returns the type of path. This is used in case we need to identify the type of BezierCurve
     * this is.
     *
     * @return returns the type of path.
     */
    @Override
    public String pathType() {
        return "point";
    }

    @Override
    public double getClosestPoint(Pose pose, int searchLimit, double initialTValueGuess) {
        return 1;
    }

    @Override
    public double getT(double pathCompletion) {
        return 1;
    }

    @Override
    public double getPathCompletion(double t) {
        return 1;
    }

    @Override
    public void initialize() {
        if (initialized) return;
        initialized = true;
        if (pose == null && !futureControlPoints.isEmpty()) pose = futureControlPoints.get(0).getPose();
        length = approximateLength();
        super.initializePanelsDrawingPoints();
    }
}
