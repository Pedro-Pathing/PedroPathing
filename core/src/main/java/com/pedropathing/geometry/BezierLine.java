package com.pedropathing.geometry;

import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * This is the BezierLine class. This class handles the creation of BezierLines, which is what I
 * call Bezier curves with only two control points. The parent BezierCurve class cannot handle Bezier
 * curves with less than three control points, so this class handles lines. Additionally, it makes
 * the calculations done on the fly a little less computationally expensive and more streamlined.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/9/2024
 */
public class BezierLine extends BezierCurve {
    private Pose startPoint;
    private Pose endPoint;

    private Vector endTangent;

    private double UNIT_TO_TIME;
    private double length;

    /**
     * This creates a new BezierLine with specified start and end Points.
     * This is just a line but it extends the BezierCurve class so things work.
     *
     * @param startPose start pose of the line.
     * @param endPose end pose of the line.
     */
    public BezierLine(Pose startPose, Pose endPose) {
        super();
        this.startPoint = startPose;
        this.endPoint = endPose;
        initialize();
    }

    public BezierLine(FuturePose startPose, FuturePose endPose) {
        super();

        if (startPose.initialized() && endPose.initialized()) {
            this.startPoint = startPose.getPose();
            this.endPoint = endPose.getPose();
            initialize();
        } else {
            futureControlPoints = new ArrayList<>(Arrays.asList(startPose, endPose));
            lazyInitialize = true;
        }
    }

    /**
     * This creates a new BezierLine with specified start and end Points.
     * This is just a line but it extends the BezierCurve class so things work.
     *
     * @param startPose start pose of the line.
     * @param endPose end pose of the line.
     * @param initialize whether to initialize the BezierLine immediately.
     */
    public BezierLine(Pose startPose, Pose endPose, boolean initialize) {
        super();
        this.startPoint = startPose;
        this.endPoint = endPose;

        if (initialize) {
            initialize();
        }
    }

    /**
     * This returns the unit tangent Vector at the end of the BezierLine.
     *
     * @return returns the tangent Vector.
     */
    @Override
    public Vector getEndTangent() {
        return endTangent.copy();
    }

    /**
     * This gets the length of the BezierLine.
     *
     * @return returns the length of the BezierLine.
     */
    @Override
    public double approximateLength() {
        return Math.sqrt(Math.pow(startPoint.getX() - endPoint.getX(), 2) + Math.pow(startPoint.getY() - endPoint.getY(), 2));
    }

    /**
     * This returns the Point on the Bezier line that is specified by the parametric t value.
     *
     * @param t this is the t value of the parametric line. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the Point requested.
     */
    @Override
    public Pose getPose(double t) {
        t = MathFunctions.clamp(t, 0, 1);
        return new Pose((endPoint.getX() - startPoint.getX()) * t + startPoint.getX(), (endPoint.getY() - startPoint.getY()) * t + startPoint.getY());
    }

    /**
     * This returns the curvature of the BezierLine, which is zero.
     *
     * @param t the parametric t value.
     * @return returns the curvature.
     */
    @Override
    public double getCurvature(double t) {
        return 0.0;
    }

    /**
     * This returns the derivative on the BezierLine as a Vector, which is a constant slope.
     * The t value doesn't really do anything, but it's there so I can override methods.
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the derivative requested.
     */
    @Override
    public Vector getDerivative(double t) {
        Vector returnVector = new Vector();

        returnVector.setOrthogonalComponents(endPoint.getX() - startPoint.getX(), endPoint.getY() - startPoint.getY());

        return returnVector;
    }

    /**
     * This returns the second derivative on the Bezier line, which is a zero Vector.
     * Once again, the t is only there for the override.
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the second derivative requested.
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
     * Returns the ArrayList of control points for this BezierLine.
     *
     * @return This returns the control points.
     */
    @Override
    public ArrayList<Pose> getControlPoints() {
        ArrayList<Pose> returnList = new ArrayList<>();
        returnList.add(startPoint);
        returnList.add(endPoint);
        return returnList;
    }

    /**
     * Returns the first control point for this BezierLine.
     *
     * @return This returns the Point.
     */
    @Override
    public Pose getFirstControlPoint() {
        return startPoint;
    }

    /**
     * Returns the second control point, or the one after the start, for this BezierLine.
     *
     * @return This returns the Point.
     */
    @Override
    public Pose getSecondControlPoint() {
        return endPoint;
    }

    /**
     * Returns the second to last control point for this BezierLine.
     *
     * @return This returns the Point.
     */
    @Override
    public Pose getSecondToLastControlPoint() {
        return startPoint;
    }

    /**
     * Returns the last control point for this BezierLine.
     *
     * @return This returns the Point.
     */
    @Override
    public Pose getLastControlPoint() {
        return endPoint;
    }

    /**
     * Returns the length of this BezierLine.
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
        return "line";
    }

    @Override
    public double getClosestPoint(Pose pose, int searchLimit, double initialTValueGuess) {
        Vector BA = new Vector(getLastControlPoint().minus(getFirstControlPoint()));
        Vector PA = new Vector(pose.minus(getFirstControlPoint()));

        return MathFunctions.clamp(BA.dot(PA) / Math.pow(BA.getMagnitude(), 2), 0, 1);
    }

    @Override
    public void initialize() {
        if (initialized) return;
        if ((startPoint == null || endPoint == null) && !futureControlPoints.isEmpty()) {
            if (startPoint == null) {
                startPoint = futureControlPoints.get(0).getPose();
            }
            if (endPoint == null) {
                endPoint = futureControlPoints.get(1).getPose();
            }
        }
        initialized = true;
        length = approximateLength();
        UNIT_TO_TIME = 1 / length;
        endTangent = getDerivative(1).normalize();
        super.initializePanelsDrawingPoints();
    }

    @Override
    public double getPathCompletion(double t) {
        return MathFunctions.clamp(t, 0, 1);
    }

    @Override
    public double getT(double pathCompletion) {
        return MathFunctions.clamp(pathCompletion, 0, 1);
    }
}
