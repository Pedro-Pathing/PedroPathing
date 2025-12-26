package com.pedropathing.paths;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Curve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import java.util.ArrayList;

/**
 * This is the Path class. This class handles containing information on the actual path the Follower
 * will follow, not just the shape of the path that the BezierCurve class handles. This contains
 * information on the stop criteria for a Path, the heading interpolation, and deceleration.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Havish Sripada - 12808 RevAmped Robotics
 * @version 1.0, 3/10/2024
 */
public class Path {
    private final Curve curve;
    private PathConstraints constraints;
    private double closestPointCurvature;
    private double closestPointTValue = 0;
    private HeadingInterpolator headingInterpolator = HeadingInterpolator.tangent;
    private Vector closestPointTangentVector;
    private Vector closestPointNormalVector;
    private Pose closestPose;

    /**
     * Creates a new Path from a BezierCurve. The default heading interpolation is tangential.
     *
     * @param curve the BezierCurve.
     * @param constraints the constraints.
     */
    public Path(Curve curve, PathConstraints constraints) {
        this.curve = curve;
        setConstraints(constraints);
    }

    /**
     * Creates a new Path from a BezierCurve. The default heading interpolation is tangential.
     *
     * @param curve the BezierCurve.
     */
    public Path(Curve curve) {
        this(curve, PathConstraints.defaultConstraints);
    }

    public Path() {
        this(new BezierCurve(new Pose(), new Pose(), new Pose()), PathConstraints.defaultConstraints);
    }

    /**
     * This sets the heading interpolation to linear with a specified start heading and end heading
     * for the Path. This will interpolate across the entire length of the Path, so there may be
     * some issues with end heading accuracy and precision if this is used. If a precise end heading
     * is necessary, then use the setLinearHeadingInterpolation(double startHeading,
     * double endHeading, double endTime) method.
     *
     * @param startHeading The start of the linear heading interpolation.
     * @param endHeading   The end of the linear heading interpolation.
     *                     This will be reached at the end of the Path if no end time is specified.
     */
    public void setLinearHeadingInterpolation(double startHeading, double endHeading) {
        this.headingInterpolator = HeadingInterpolator.linear(startHeading, endHeading);
    }

    /**
     * This sets the heading interpolation to linear with a specified start heading and end heading
     * for the Path. This will interpolate from the start of the Path to the specified end time.
     * This ensures high accuracy and precision than interpolating across the entire Path. However,
     * interpolating too quickly can cause undesired oscillations and inaccuracies of its own, so
     * generally interpolating to something like 0.8 of your Path should work best.
     *
     * @param startHeading The start of the linear heading interpolation.
     * @param endHeading   The end of the linear heading interpolation.
     *                     This will be reached at the end of the Path if no end time is specified.
     * @param endTime      The end time on the Path that the linear heading interpolation will finish.
     *                     This value ranges from [0, 1] since Bezier curves are parametric functions.
     */
    public void setLinearHeadingInterpolation(double startHeading, double endHeading, double endTime) {
        this.headingInterpolator = HeadingInterpolator.linear(startHeading, endHeading, endTime);
    }

    /**
     * This sets the heading interpolation to linear with a specified start heading and end heading
     * for the Path. This will interpolate from the start of the Path to the specified end time.
     * This ensures high accuracy and precision than interpolating across the entire Path. However,
     * interpolating too quickly can cause undesired oscillations and inaccuracies of its own, so
     * generally interpolating to something like 0.8 of your Path should work best.
     *
     * @param startHeading The start of the linear heading interpolation.
     * @param endHeading   The end of the linear heading interpolation.
     *                     This will be reached at the end of the Path if no end time is specified.
     * @param endTime      The end time on the Path that the linear heading interpolation will finish.
     *                     This value ranges from [0, 1] since Bezier curves are parametric functions.
     */
    public void setLinearHeadingInterpolation(double startHeading, double endHeading, double endTime, boolean reversed) {
        this.headingInterpolator = HeadingInterpolator.linear(startHeading, endHeading, endTime);
        if (reversed) {
            reverseHeadingInterpolation();
        }
    }

    /**
     * This sets the heading interpolation to maintain a constant heading.
     *
     * @param setHeading the constant heading for the Path.
     */
    public void setConstantHeadingInterpolation(double setHeading) {
        this.headingInterpolator = HeadingInterpolator.constant(setHeading);
    }

    /**
     * This gets the closest point from a specified pose to the BezierCurve with a Newton search
     * that is limited to some specified step limit.
     * This will use the initial guess for the t-value as a starting point.
     *
     * @param pose        the pose.
     * @param searchLimit the maximum number of iterations to run.
     * @param initialTValueGuess the initial guess for the t-value of the pose
     * @return returns the closest Point.
     */
    public PathPoint getClosestPoint(Pose pose, int searchLimit, double initialTValueGuess) {
        initialTValueGuess = curve.getClosestPoint(pose, searchLimit, initialTValueGuess);
        return getPoseInformation(initialTValueGuess);
    }

    /**
     * This gets the closest point from a specified pose to the BezierCurve with a Newton search
     * @param pose the pose
     * @param searchLimit the maximum number of iterations to run
     * @return the closest point
     */
    public PathPoint getClosestPoint(Pose pose, int searchLimit) {
        return getClosestPoint(pose, searchLimit, closestPointTValue);
    }

    /**
     * This gets the point on the path closest to the specified pose.
     * @param pose the pose to find the closest point to
     * @return the closest point t-value
     */
    public PathPoint getClosestPoint(Pose pose) {
        return getClosestPoint(pose, constraints.getBEZIER_CURVE_SEARCH_LIMIT());
    }

    /**
     * This gets the closest Pose on the BezierCurve to the current Pose.
     */
    public PathPoint getClosestPose() {
        return new PathPoint(closestPointTValue, closestPose, closestPointTangentVector);
    }

    /**
     * This updates the closest Pose on the BezierCurve to the current Pose.
     * It will search for the closest point within a specified limit.
     *
     * @param currentPose the current pose of the robot.
     * @param searchLimit the maximum number of iterations to run.
     * @return returns the closest Point on the BezierCurve.
     */
    public PathPoint updateClosestPose(Pose currentPose, int searchLimit) {
        PathPoint closestPoint = getClosestPoint(currentPose, searchLimit);
        closestPointTValue = closestPoint.getTValue();
        closestPose = closestPoint.getPose();
        closestPointTangentVector = closestPoint.getTangentVector();
        closestPointNormalVector = curve.getNormalVector(closestPointTValue);
        closestPointCurvature = curve.getCurvature(closestPointTValue);
        return closestPoint;
    }

    /**
     * This updates the closest pose to the specified pose.
     * @param currentPose the pose to find the closest point to
     * @return the closest point
     */
    public PathPoint updateClosestPose(Pose currentPose) {
        return updateClosestPose(currentPose, constraints.getBEZIER_CURVE_SEARCH_LIMIT());
    }

    /**
     * This returns the distance to the specified pose.
     * @return the distance to the specified pose
     */
    public double getDistanceTraveled(double t) {
        return getPathCompletion(t) * curve.length();
    }

    /**
     * This returns the path completion by distance
     * @return the current path completion
     */
    public double getPathCompletion(double t) {
        return curve.getPathCompletion(t);
    }

    /**
     * This returns the path completion by distance at the closest point.
     * @return the current path completion
     */

    public double getPathCompletion() {
        return curve.getPathCompletion(closestPointTValue);
    }

    /**
     * This returns the distance traveled by the robot along the Path.
     * This is calculated by multiplying the path completion by the length of the BezierCurve.
     *
     * @return returns the distance traveled.
     */
    public double getDistanceTraveled() {
        return getDistanceTraveled(closestPointTValue);
    }

    /**
     * This returns the distance remaining to the end of the Path.
     * @param t the t-value to calculate the distance remaining from.
     * @return returns the distance remaining to the end of the Path.
     */
    public double getDistanceRemaining(double t) {
        return curve.length() - getDistanceTraveled(t);
    }

    /**
     * This returns the distance remaining to the end of the Path.
     * @return returns the distance remaining to the end of the Path.
     */
    public double getDistanceRemaining() {
        return getDistanceRemaining(closestPointTValue);
    }

    /**
     * This returns the t-value of the Path at a specified path completion.
     * @param pathCompletion the path completion to get the t-value for.
     * @return returns the t-value of the Path at the specified path completion.
     */
    public double getTFromPathCompletion(double pathCompletion) {
        return curve.getT(pathCompletion);
    }

    /**
     * Reverse the direction of the heading interpolation.
     */
    public void reverseHeadingInterpolation() {
        this.headingInterpolator = headingInterpolator.reverse();
    }

    /**
     * This sets the heading interpolation to tangential.
     */
    public void setTangentHeadingInterpolation() {
        this.headingInterpolator = HeadingInterpolator.tangent;
    }

    /**
     * This gets the tangent Vector at the specified t-value.
     * @param tvalue the t-value to get the tangent Vector at.
     */
    public Vector getTangentVector(double tvalue) {
        return curve.getDerivative(tvalue);
    }

    /**
     * This returns the unit tangent Vector at the end of the BezierCurve.
     *
     * @return returns the end tangent Vector.
     */
    public Vector getEndTangent() {
        return curve.getEndTangent();
    }

    /**
     * NOTE: THIS DOES NOT RETURN HEADING. This returns the point on the Bezier curve that is specified by the parametric t value. A
     * Bezier curve is a parametric function that returns points along it with t ranging from [0, 1],
     * with 0 being the beginning of the curve and 1 being at the end. The Follower will follow
     * BezierCurves from 0 to 1, in terms of t.
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the point requested.
     */
    public Pose getPoint(double t) {
        return curve.getPose(t);
    }

    /**
     * This returns the Pose at a specified t-value. The Pose contains the x and y position, as well
     * as the heading goal at that t-value.
     *
     * @param t the specified t-value.
     * @return returns the Pose at the specified t-value.
     */
    public Pose getPose(double t) {
        Pose position = curve.getPose(t);
        return new Pose(position.getX(), position.getY(), getHeadingGoal(t));
    }

    /**
     * This returns the Pose information at a specified t-value as a PathPoint.
     *
     * @param t the specified t-value.
     * @return returns the PathPoint containing the Pose and tangent Vector at the specified t-value.
     */
    public PathPoint getPoseInformation(double t) {
        return new PathPoint(t, getPose(t), getTangentVector(t));
    }

    /**
     * This returns the t-value of the closest Point on the BezierCurve.
     *
     * @return returns the closest Point t-value.
     */
    public double getClosestPointTValue() {
        return closestPointTValue;
    }

    /**
     * This returns the approximated length of the BezierCurve.
     *
     * @return returns the length of the BezierCurve.
     */
    public double length() {
        return curve.length();
    }

    /**
     * This returns the curvature of the BezierCurve at a specified t-value.
     *
     * @param t the specified t-value.
     * @return returns the curvature of the BezierCurve at the specified t-value.
     */
    public double getCurvature(double t) {
        return curve.getCurvature(t);
    }

    /**
     * This returns the curvature of the BezierCurve at the closest Point.
     *
     * @return returns the curvature of the BezierCurve at the closest Point.
     */
    public double getClosestPointCurvature() {
        return closestPointCurvature;
    }

    /**
     * This returns the normal Vector at the closest Point.
     *
     * @return returns the normal Vector at the closest Point.
     */
    public Vector getClosestPointNormalVector() {
        return closestPointNormalVector.copy();
    }

    /**
     * This returns the tangent Vector at the closest Point.
     *
     * @return returns the tangent Vector at the closest Point.
     */
    public Vector getClosestPointTangentVector() {
        return closestPointTangentVector.copy();
    }

    /**
     * This returns the heading goal at the closest Point.
     *
     * @return returns the heading goal at the closest Point.
     */
    public double getClosestPointHeadingGoal() {
        return closestPose.getHeading();
    }

    /**
     * This returns the heading goal at the closest Point.
     *
     * @param closestPoint the closest Point to get the heading goal for.
     * @return returns the heading goal at the closest Point.
     */
    public double getHeadingGoal(PathPoint closestPoint) {
        return this.headingInterpolator.interpolate(closestPoint);
    }

    /**
     * This returns the heading goal at a specified t-value.
     *
     * @param t the specified t-value.
     * @return returns the heading goal at the specified t-value.
     */
    public double getHeadingGoal(double t) {
        return this.headingInterpolator.interpolate(new PathPoint(t, curve.getPose(t), getTangentVector(t)));
    }

    /**
     * This sets the heading interpolation to a custom HeadingInterpolator.
     *
     * @param interpolator the custom HeadingInterpolator to set.
     */
    public void setHeadingInterpolation(HeadingInterpolator interpolator) {
        this.headingInterpolator = interpolator;
    }

    /**
     * This returns if the robot is at the end of the Path, according to the parametric t-value.
     *
     * @return returns if at end.
     */
    public boolean isAtParametricEnd() {
        return curve.atParametricEnd(closestPointTValue);
    }

    /**
     * This returns if the robot is at the beginning of the Path, according to the parametric t-value.
     *
     * @return returns if at start.
     */
    public boolean isAtParametricStart() {
        return closestPointTValue <= 1 - constraints.getTValueConstraint();
    }

    /**
     * Returns the ArrayList of control points for this BezierCurve.
     *
     * @return This returns the control points.
     */
    public ArrayList<Pose> getControlPoints() {
        return curve.getControlPoints();
    }

    /**
     * Returns the first control point for this BezierCurve.
     *
     * @return This returns the Point.
     */
    public Pose getFirstControlPoint() {
        return curve.getFirstControlPoint();
    }

    /**
     * Returns the second control point, or the one after the start, for this BezierCurve.
     *
     * @return This returns the Point.
     */
    public Pose getSecondControlPoint() {
        return curve.getSecondControlPoint();
    }

    /**
     * Returns the second to last control point for this BezierCurve.
     *
     * @return This returns the Point.
     */
    public Pose getSecondToLastControlPoint() {
        return curve.getSecondToLastControlPoint();
    }

    /**
     * Returns the last control point for this BezierCurve.
     *
     * @return This returns the Point.
     */
    public Pose getLastControlPoint() {
        return curve.getLastControlPoint();
    }

    /**
     * This sets the path's deceleration factor in terms of the natural deceleration of the robot
     * when power is cut from the drivetrain.
     *
     * @param set This sets the multiplier.
     */
    public void setBrakingStrength(double set) {
        constraints.setBrakingStrength(set);
    }

    /**
     * This sets the multiplier on where the robot starts decelerating along the path.
     * A higher number will decelerate earlier and a later number will decelerate later.
     *
     * @param set This sets the multiplier.
     */
    public void setBrakingStart(double set) {
        constraints.setBrakingStart(set);
    }


    /**
     * This sets the velocity stop criteria. When velocity is below this amount, then this is met.
     *
     * @param set This sets the velocity end constraint.
     */
    public void setVelocityConstraint(double set) {
        constraints.setVelocityConstraint(set);
    }

    /**
     * This sets the translational stop criteria. When the translational error, or how far off the
     * end point the robot is, goes below this, then the translational end criteria is met.
     *
     * @param set This sets the translational end constraint.
     */
    public void setTranslationalConstraint(double set) {
        constraints.setTranslationalConstraint(set);
    }

    /**
     * This sets the heading stop criteria. When the heading error is less than this amount, then
     * the heading end criteria is met.
     *
     * @param set This sets the heading end constraint.
     */
    public void setHeadingConstraint(double set) {
        constraints.setHeadingConstraint(set);
    }

    /**
     * This sets the parametric end criteria. When the t-value of the closest Point on the Path is
     * greater than this amount, then the parametric end criteria is met.
     *
     * @param set This sets the t-value end constraint.
     */
    public void setTValueConstraint(double set) {
       constraints.setTValueConstraint(set);
    }

    /**
     * This sets the Path end timeout. If the Path is at its end parametrically, then the Follower
     * has this many milliseconds to correct before the Path gets ended anyways.
     *
     * @param set This sets the Path end timeout.
     */
    public void setTimeoutConstraint(double set) {
        constraints.setTimeoutConstraint(set);
    }

    /**
     * This gets the deceleration multiplier.
     *
     * @return This returns the deceleration multiplier.
     */
    public double getBrakingStrength() {
        return constraints.getBrakingStrength();
    }

    /**
     * This gets the deceleration start multiplier.
     *
     * @return This returns the deceleration start multiplier.
     */
    public double getBrakingStartMultiplier() {
        return constraints.getBrakingStart();
    }

    /**
     * This gets the velocity stop criteria.
     *
     * @return This returns the velocity stop criteria.
     */
    public double getPathEndVelocityConstraint() {
        return constraints.getVelocityConstraint();
    }

    /**
     * This gets the translational stop criteria.
     *
     * @return This returns the translational stop criteria.
     */
    public double getPathEndTranslationalConstraint() {
        return constraints.getTranslationalConstraint();
    }

    /**
     * This gets the heading stop criteria.
     *
     * @return This returns the heading stop criteria.
     */
    public double getPathEndHeadingConstraint() {
        return constraints.getHeadingConstraint();
    }

    /**
     * This gets the parametric end criteria.
     *
     * @return This returns the parametric end criteria.
     */
    public double getPathEndTValueConstraint() {
        return constraints.getTValueConstraint();
    }

    /**
     * This gets the Path end correction time.
     *
     * @return This returns the Path end correction time.
     */
    public double getPathEndTimeoutConstraint() {
        return constraints.getTimeoutConstraint();
    }

    /**
     * Returns the type of path. This is used in case we need to identify the type of BezierCurve
     * this is.
     *
     * @return returns the type of path.
     */
    public String pathType() {
        return curve.pathType();
    }

    /**
     * This returns a 2D Array of doubles containing the x and y positions of points to draw on
     * Panels.
     *
     * @return returns the 2D Array to draw on Panels
     */
    public double[][] getPanelsDrawingPoints() {
        return curve.getPanelsDrawingPoints();
    }

    /**
     * This gets the heading interpolation.
     * @return This returns the heading interpolation.
     */
    public HeadingInterpolator getHeadingInterpolator() {
        return headingInterpolator;
    }

    /**
     * This gets the BezierCurve that this Path is based on.
     *
     * @return returns the BezierCurve.
     */
    public Pose endPose() {
        Pose lastControlPoint = curve.getLastControlPoint();
        return new Pose(lastControlPoint.getX(), lastControlPoint.getY(),
            getHeadingGoal(new PathPoint(1, lastControlPoint, curve.getEndTangent())));
    }

    /**
     * This returns if the Path is reversed.
     */
    public Path getReversed() {
        return new Path(curve.getReversed());
    }

    /**
     * This sets the constraints for the Path. This includes the zero power acceleration multiplier,
     * path end velocity constraint, path end translational constraint, path end heading constraint,
     * path end t-value constraint, path end timeout constraint, and the Bezier curve search limit.
     *
     * @param constraints the PathConstraints to set.
     */
    public void setConstraints(PathConstraints constraints) {
        this.constraints = constraints.copy();

        if (curve != null)
            curve.setPathConstraints(this.constraints);
    }

    /**
     * This gets the PathConstraints for the Path.
     *
     * @return returns the PathConstraints.
     */
    public PathConstraints getConstraints() {
        return constraints;
    }

    /**
     * Gets the curve associated with this path
     * @return the curve associated with this path
     */
    public Curve getCurve() {
        return curve;
    }

    /**
     * Initializes the curve associated with this path.
     * This is necessary to ensure that the curve is ready for use.
     */
    public void init() {
        curve.initialize();
        headingInterpolator.init();
    }
}