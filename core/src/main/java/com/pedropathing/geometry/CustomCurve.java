package com.pedropathing.geometry;

import com.pedropathing.math.AbstractBijectiveMap;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * This is the CustomCurve class. This class can be used to generate custom curves for the robot to follow.
 * @author Havish Sripada - 12808 RevAmped Robotics
 *
 * Modified by J Kenney to accomodate a wider variety of custom curves and to prevent
 * problems that can arise when repeat following a path.
 */
public abstract class CustomCurve implements Curve {
    protected ArrayList<Pose> controlPoints;
    protected PathConstraints pathConstraints;
    private List<FuturePose> futureControlPoints;
    private boolean initialized = false;
    private double length = 0;
    private double[][] panelsDrawingPoints;
    protected AbstractBijectiveMap.NumericBijectiveMap completionMap = new AbstractBijectiveMap.NumericBijectiveMap();

    /*
     * Set this flag each time the curve is initialized, to indicate that the initialTValueGuess for
     * getClosestPoint needs to be reset. Otherwise, following the same curve a second time may not work.
     */
    private boolean resetClosestPointFlag = true;

    /*
     * Arbitrary data that may be needed to generate the custom curve
     */
    protected Object curveData;

    /*
     * Add constructors that accept arbitrary data, 'curveData' as parameters. For example, one could pass a
     * double[2] array that specifies the beginning and ending direction of travel for a spline.
     */

    public CustomCurve(List<Pose> controlPoints, PathConstraints pathConstraints, Object curveData) {
        this.pathConstraints = pathConstraints;
        this.controlPoints = new ArrayList<>(controlPoints);

        this.curveData = curveData;

        // Initialize the curve with the provided control points
        initialize();
    }

    public CustomCurve(List<Pose> controlPoints, PathConstraints pathConstraints){
        this(controlPoints, pathConstraints, null);
    }

    public CustomCurve(PathConstraints pathConstraints, Object curveData, Pose... controlPoints){
        this(new ArrayList<>(Arrays.asList(controlPoints)), pathConstraints, curveData);
    }

    public CustomCurve(Pose... controlPoints) {
        this(PathConstraints.defaultConstraints, null, controlPoints);
    }

    public CustomCurve(PathConstraints pathConstraints, FuturePose... controlPoints) {
        this(pathConstraints, null, controlPoints);
    }

    public CustomCurve(FuturePose... controlPoints){
        this(PathConstraints.defaultConstraints, null, controlPoints);
    }

    public CustomCurve(PathConstraints constraints, Object curveData, FuturePose... controlPoints) {
        this.pathConstraints = constraints;
        this.controlPoints = new ArrayList<>();

        this.curveData = curveData;

        boolean lazyInitialize = false;
        ArrayList<Pose> initializedControlPoints = new ArrayList<>();
        for (FuturePose pose : controlPoints) {
            if (!pose.initialized()) {
                lazyInitialize = true;
                break;
            }

            initializedControlPoints.add(pose.getPose());
        }

        if (lazyInitialize) {
            this.controlPoints = new ArrayList<>();
            this.futureControlPoints = new ArrayList<>(Arrays.asList(controlPoints));
        } else {
            this.controlPoints = initializedControlPoints;
            initialize();
        }
    }

    @Override
    public ArrayList<Pose> getControlPoints() {
        return controlPoints;
    }

    @Override
    public void setPathConstraints(PathConstraints pathConstraints) {
        this.pathConstraints = pathConstraints;
    }

    @Override
    public PathConstraints getPathConstraints() {
        return pathConstraints;
    }

    /**
     * This is to be overridden by subclasses for specific initialization logic.
     */
    public void initialization() {
        // To be overridden by subclasses for specific initialization logic
    };

    @Override
    public void initialize() {
        /*
         * Set the flag so that the initialTValueGuess for getClosestPoint will be reset during the next
         * call to getClosestPoint.
         */
        resetClosestPointFlag = true;

        if (initialized) return;

        if (controlPoints.isEmpty() && !futureControlPoints.isEmpty()) {
            // If control points are not initialized, use future control points
            for (FuturePose futurePose : futureControlPoints) {
                controlPoints.add(futurePose.getPose());
            }
            futureControlPoints.clear();
        }

        initialization();

        initialized = true;
        length = approximateLength();
        initializePanelsDrawingPoints();
    }

    /**
     * This creates the Array that holds the Points to draw on Panels.
     */
    public void initializePanelsDrawingPoints() {
        int DASHBOARD_DRAWING_APPROXIMATION_STEPS = 100;
        this.panelsDrawingPoints = new double[2][DASHBOARD_DRAWING_APPROXIMATION_STEPS + 1];
        for (int i = 0; i <= DASHBOARD_DRAWING_APPROXIMATION_STEPS; i++) {
            Pose currentPoint = this.getPose(i/(double) (DASHBOARD_DRAWING_APPROXIMATION_STEPS));
            this.panelsDrawingPoints[0][i] = currentPoint.getX();
            this.panelsDrawingPoints[1][i] = currentPoint.getY();
        }
    }

    @Override
    public double[][] getPanelsDrawingPoints() {
        return panelsDrawingPoints;
    }

    /**
     * This approximates the length of the BezierCurve in APPROXIMATION_STEPS number of steps. It's
     * like a Riemann's sum, but for a parametric function's arc length.
     *
     * @return returns the approximated length of the BezierCurve.
     */
    public double approximateLength() {
        Pose previousPoint = getPose(0);
        Pose currentPoint;
        double approxLength = 0;
        int APPROXIMATION_STEPS = 1000;
        for (int i = 1; i <= APPROXIMATION_STEPS; i++) {
            double t = i/(double) APPROXIMATION_STEPS;
            currentPoint = getPose(t);
            approxLength += previousPoint.distanceFrom(currentPoint);
            previousPoint = currentPoint;
            completionMap.put(t, approxLength);
        }
        return approxLength;
    }

    @Override
    public double length() {
        return length;
    }

    @Override
    public abstract String pathType();

    @Override
    public abstract CustomCurve getReversed();

    @Override
    public double getPathCompletion(double t) {
        if (length == 0) return 0.0;
        return completionMap.interpolateKey(t) / length;
    }

    public double getT(double pathCompletion) {
        return completionMap.interpolateValue(pathCompletion);
    }

    @Override
    public Vector getNormalVector(double t) {
        t = MathFunctions.clamp(t, 0, 1);

        Vector v = getDerivative(t);         // velocity
        Vector a = getSecondDerivative(t);   // acceleration

        double vMag = v.getMagnitude();
        if (vMag == 0) return new Vector(0, 0); // no tangent direction

        Vector tangent = v.normalize();

        // Normal component of acceleration
        Vector aNormal = a.minus(tangent.times(a.dot(tangent)));

        double aNormalMag = aNormal.getMagnitude();
        if (aNormalMag == 0) return new Vector(0, 0); // straight line, no principal normal

        return aNormal.normalize();
    }

    /*
     * Modify the default getClosestPoint method so that the initialTValueGuess gets reset
     * to a rough estimate of closest, if the curve has been initialized since that last call
     * to the getClosestPoint method.
     */
    @Override
    public double getClosestPoint(Pose pose, int searchLimit, double initialTValueGuess){
        if (resetClosestPointFlag){
            resetClosestPointFlag = false;
            initialTValueGuess = 0.0;
            Pose pTest = this.getPose(0);
            double closestSquare = (pTest.getX()-pose.getX())*(pTest.getX()-pose.getX())
                    + (pTest.getY()-pose.getY())*(pTest.getY()-pose.getY());
            double[] searchEstimates =
                    new double[]{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
            for (int k=0; k<searchEstimates.length; k++){
                pTest = this.getPose(searchEstimates[k]);
                double distSquared = (pTest.getX()-pose.getX())*(pTest.getX()-pose.getX())
                        + (pTest.getY()-pose.getY())*(pTest.getY()-pose.getY());
                if (distSquared < closestSquare){
                    initialTValueGuess = searchEstimates[k];
                    closestSquare = distSquared;
                }
            }
        }
        return Curve.super.getClosestPoint(pose, searchLimit, initialTValueGuess);
    }
}
