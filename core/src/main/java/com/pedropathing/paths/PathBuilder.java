package com.pedropathing.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Curve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.callbacks.ParametricCallback;
import com.pedropathing.paths.callbacks.PathCallback;
import com.pedropathing.paths.callbacks.PoseCallback;
import com.pedropathing.paths.callbacks.TemporalCallback;
import com.pedropathing.util.FiniteRunAction;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * This is the PathBuilder class. This class makes it easier to create PathChains, so you don't have
 * to individually create Path instances to create a PathChain. A PathBuilder can be accessed
 * through running the pathBuilder() method on an instance of the Follower class, or just creating
 * an instance of PathBuilder regularly.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Havish Sripada - 12808 RevAmped Robotics
 * @version 1.0, 3/11/2024
 */
public class PathBuilder {
    private final ArrayList<Path> paths = new ArrayList<>();
    private PathChain.DecelerationType decelerationType = PathChain.DecelerationType.LAST_PATH;
    private final ArrayList<PathCallback> callbacks = new ArrayList<>();
    private PathConstraints constraints;
    private HeadingInterpolator headingInterpolator;
    private final Follower follower;

    /**
     * This is an constructor for the PathBuilder class so it can get started with specific constraints.
     * The PathBuilder allows for easier construction of PathChains.
     * The proper usage is using an instance of the Follower class:
     * Follower follower = new Follower(hardwareMap);
     * Then calling "follower.pathBuilder.[INSERT PATH BUILDING METHODS].build();
     * Of course, you can split up the method calls onto separate lines for readability.
     */
    public PathBuilder(Follower follower, PathConstraints constraints) {
        this.follower = follower;
        this.constraints = constraints;
    }

    /**
     * This is an empty constructor for the PathBuilder class so it can get started, it will use the default constraints.
     * The PathBuilder allows for easier construction of PathChains.
     * The proper usage is using an instance of the Follower class:
     * Follower follower = new Follower(hardwareMap);
     * Then calling "follower.pathBuilder.[INSERT PATH BUILDING METHODS].build();
     * Of course, you can split up the method calls onto separate lines for readability.
     */
    public PathBuilder(Follower follower) {
        this(follower, PathConstraints.defaultConstraints);
    }

    /**
     * This adds a Path to the PathBuilder.
     *
     * @param path The Path being added.
     * @return This returns itself with the updated data.
     */
    public PathBuilder addPath(Path path) {
        path.setConstraints(constraints);
        this.paths.add(path);
        return this;
    }

    /**
     * This adds a default Path defined by a specified BezierCurve to the PathBuilder.
     *
     * @param curve This curve is turned into a Path and added.
     * @return This returns itself with the updated data.
     */
    public PathBuilder addPath(Curve curve) {
        return addPath(new Path(curve, constraints));
    }

    /**
     * This adds multiple Paths to the PathBuilder
     * @param paths Vararg of Paths being added
     * @return This returns itself with the updated data.
     */
    public PathBuilder addPaths(Path... paths){
        for (Path path: paths) {
            path.setConstraints(constraints);
            this.paths.add(path);
        }
        return this;
    }

    /**
     * This adds multiple curves wrapped in the default Path defined by the curve to the PathBuilder
     * @param curves Vararg of Curves being added
     * @return This returns itself with the updated data.
     */
    public PathBuilder addPaths(Curve... curves){
        for (Curve curve: curves) {
            this.paths.add(new Path(curve, constraints));
        }
        return this;
    }

    /**
     * Automagically generate bézier curves through each given point and add to path
     * @param prevPoint the point prior to the start point
     * @param startPoint start point of the curve chain
     * @param tension controls tangents' generation magnitude
     * @param points other points
     * @return This returns itself with the updated data.
     */
    public PathBuilder curveThrough(Pose prevPoint, Pose startPoint, double tension, Pose... points){
        //guard against points being zero length (which means the curve doesn't have an end point)
        if (points.length == 0) {
            throw new IllegalArgumentException("Points array must contain at least one point to curve through.");
        }
        ArrayList<Pose> poses = new ArrayList<>();

        poses.add(prevPoint);
        poses.add(startPoint);

        poses.addAll(Arrays.asList(points));

        // auto calculate new end point to generate a valid tangent
        Pose diff = poses.get(poses.size() - 1).minus(poses.get(poses.size() - 2));
        poses.add(poses.get(poses.size() - 1).plus(diff));

        double scaledTension = tension / 3d;

        List<ArrayList<Pose>> controlPoints = new ArrayList<>();
        for (int i = 1; i < poses.size() - 2; i++) {
            controlPoints.add(catmullToBezier(scaledTension, poses.get(i - 1), poses.get(i), poses.get(i + 1), poses.get(i + 2)));
        }

        BezierCurve[] curves = new BezierCurve[controlPoints.size()];
        for (int i = 0; i < curves.length; i++) {
            curves[i] = new BezierCurve(controlPoints.get(i), this.constraints);
        }

        return addPaths(curves);
    }

    /**
     * Automagically generate bézier curves through each given point and add to path.
     * This method starts the first curve from the last path's end point.
     * @param tension controls tangents' generation magnitude
     * @param points points to curve through
     * @return This returns itself with the updated data.
     */
    public PathBuilder curveThrough(double tension, Pose... points){
        Pose prevPoint;
        Pose startPoint;

        if (!this.paths.isEmpty()){
            prevPoint = this.paths.get(paths.size() - 1).getFirstControlPoint();
            startPoint = this.paths.get(paths.size() - 1).getLastControlPoint();
        } else {
            // fallback if the last path doesn't exist
            startPoint = this.follower.getPoseTracker().getPreviousPose().copy();
            // the first element of points must exist
            prevPoint = startPoint.minus(points[0].minus(startPoint));
        }

        return curveThrough(prevPoint, startPoint, tension, points);
    }

    /**
     * Converts catmull rom spline points into cubic bezier control points
     * @param scaledTension tension / 3
     * @param p0 previous pose
     * @param p1 current pose
     * @param p2 next pose
     * @param p3 next pose of the next pose
     * @return list of cubic bezier control points
     */
    private ArrayList<Pose> catmullToBezier(double scaledTension, Pose p0, Pose p1, Pose p2, Pose p3){
        ArrayList<Pose> output = new ArrayList<>();
        output.add(p1);
        output.add(p1.plus((p2.minus(p0)).times(scaledTension)));
        output.add(p2.minus((p3.minus(p1)).times(scaledTension)));
        output.add(p2);

        return output;
    }

    /**
     * This sets a linear heading interpolation on the last Path added to the PathBuilder.
     *
     * @param startHeading The start of the linear heading interpolation.
     * @param endHeading The end of the linear heading interpolation.
     *         This will be reached at the end of the Path if no end t-value is specified.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setLinearHeadingInterpolation(double startHeading, double endHeading) {
        this.paths.get(paths.size() - 1).setLinearHeadingInterpolation(startHeading, endHeading);
        return this;
    }

    /**
     * This sets a global linear heading interpolation.
     *
     * @param startHeading The start of the linear heading interpolation.
     * @param endHeading The end of the linear heading interpolation.
     *         This will be reached at the end of the Path if no end t-value is specified.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setGlobalLinearHeadingInterpolation(double startHeading, double endHeading) {
        headingInterpolator = HeadingInterpolator.linear(startHeading, endHeading);
        return this;
    }

    /**
     * This sets a linear heading interpolation on the last Path added to the PathBuilder.
     *
     * @param startHeading The start of the linear heading interpolation.
     * @param endHeading The end of the linear heading interpolation.
     *         This will be reached at the end of the Path if no end t-value is specified.
     * @param endTime The end t-value on the Path that the linear heading interpolation will end.
     *         This value goes from [0, 1] since Bezier curves are parametric functions.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setLinearHeadingInterpolation(double startHeading, double endHeading, double endTime) {
        this.paths.get(paths.size() - 1).setLinearHeadingInterpolation(startHeading, endHeading, endTime);
        return this;
    }

    /**
     * This sets a global linear heading interpolation.
     *
     * @param startHeading The start of the linear heading interpolation.
     * @param endHeading The end of the linear heading interpolation.
     *         This will be reached at the end of the Path if no end t-value is specified.
     * @param endTime The end t-value on the Path that the linear heading interpolation will end.
     *         This value goes from [0, 1] since Bezier curves are parametric functions.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setGlobalLinearHeadingInterpolation(double startHeading, double endHeading, double endTime) {
        headingInterpolator = HeadingInterpolator.linear(startHeading, endHeading, endTime);
        return this;
    }

    /**
     * This sets a linear heading interpolation on the last Path added to the PathBuilder.
     *
     * @param startHeading The start of the linear heading interpolation.
     * @param endHeading The end of the linear heading interpolation.
     *         This will be reached at the end of the Path if no end t-value is specified.
     * @param startTime The start t-value on the Path that the linear heading interpolation will start.
     * @param endTime The end t-value on the Path that the linear heading interpolation will end.
     *         This value goes from [0, 1] since Bezier curves are parametric functions.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setLinearHeadingInterpolation(double startHeading, double endHeading, double endTime, double startTime) {
        HeadingInterpolator interpolator = HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, startTime, HeadingInterpolator.constant(startHeading)),
                HeadingInterpolator.PiecewiseNode.linear(startTime, endTime, startHeading, endHeading)
        );

        this.paths.get(paths.size() - 1).setHeadingInterpolation(interpolator);
        return this;
    }

    /**
     * This sets a global heading interpolation on the last Path added to the PathBuilder.
     *
     * @param startHeading The start of the linear heading interpolation.
     * @param endHeading The end of the linear heading interpolation.
     *         This will be reached at the end of the Path if no end t-value is specified.
     * @param startTime The start t-value on the Path that the linear heading interpolation will start.
     * @param endTime The end t-value on the Path that the linear heading interpolation will end.
     *         This value goes from [0, 1] since Bezier curves are parametric functions.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setGlobalLinearHeadingInterpolation(double startHeading, double endHeading, double endTime, double startTime) {
        headingInterpolator = HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, startTime, HeadingInterpolator.constant(startHeading)),
                HeadingInterpolator.PiecewiseNode.linear(startTime, endTime, startHeading, endHeading)
        );
        return this;
    }

    /**
     * This sets a constant heading interpolation on the last Path added to the PathBuilder.
     *
     * @param setHeading The constant heading specified.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setConstantHeadingInterpolation(double setHeading) {
        this.paths.get(paths.size() - 1).setConstantHeadingInterpolation(setHeading);
        return this;
    }

    /**
     * This sets a global constant heading interpolation.
     *
     * @param setHeading The constant heading specified.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setGlobalConstantHeadingInterpolation(double setHeading) {
        headingInterpolator = HeadingInterpolator.constant(setHeading);
        return this;
    }

    /**
     * This sets a reversed heading interpolation on the last Path added to the PathBuilder.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setReversed() {
        this.paths.get(paths.size() - 1).reverseHeadingInterpolation();
        return this;
    }

    /**
     * This sets a global reversed heading interpolation.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setGlobalReversed() {
        headingInterpolator = headingInterpolator.reverse();
        return this;
    }

    /**
     * This sets the heading interpolation to tangential on the last Path added to the PathBuilder.
     * There really shouldn't be a reason to use this since the default heading interpolation is
     * tangential but it's here.
     */
    public PathBuilder setTangentHeadingInterpolation() {
        this.paths.get(paths.size() - 1).setTangentHeadingInterpolation();
        return this;
    }

    /**
     * This sets the global heading interpolation to tangential..
     * There really shouldn't be a reason to use this since the default heading interpolation is
     * tangential but it's here.
     */
    public PathBuilder setGlobalTangentHeadingInterpolation() {
        headingInterpolator = HeadingInterpolator.tangent;
        return this;
    }

    /**
     * This sets the heading interpolation to custom on the last Path added to the PathBuilder.
     * @param function A function that describes the target heading as a function of t, the parametric variable. Use a lambda expression here.
     */
    public PathBuilder setHeadingInterpolation(HeadingInterpolator function) {
        this.paths.get(paths.size() - 1).setHeadingInterpolation(function);
        return this;
    }

    /**
     * This sets the global heading interpolation to custom.
     * @param function A function that describes the target heading as a function of t, the parametric variable. Use a lambda expression here.
     */
    public PathBuilder setGlobalHeadingInterpolation(HeadingInterpolator function) {
        this.headingInterpolator = function;
        return this;
    }

    /**
     * This sets the deceleration multiplier on the last Path added to the PathBuilder.
     *
     * @param set This sets the multiplier for the goal for the deceleration of the robot.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setBrakingStrength(double set) {
        this.paths.get(paths.size() - 1).setBrakingStrength(set);
        return this;
    }

    /**
     * This sets the breaking start
     *
     * @param set This sets the multiplier
     * @return This returns itself with the updated data.
     */
    public PathBuilder setBrakingStart(double set) {
        constraints.setBrakingStart(set);
        return this;
    }

    /**
     * This sets the path end velocity constraint on the last Path added to the PathBuilder.
     *
     * @param set This sets the path end velocity constraint.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setVelocityConstraint(double set) {
        this.paths.get(paths.size() - 1).setVelocityConstraint(set);
        return this;
    }

    /**
     * This sets the path end translational constraint on the last Path added to the PathBuilder.
     *
     * @param set This sets the path end translational constraint.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setTranslationalConstraint(double set) {
        this.paths.get(paths.size() - 1).setTranslationalConstraint(set);
        return this;
    }

    /**
     * This sets the path end heading constraint on the last Path added to the PathBuilder.
     *
     * @param set This sets the path end heading constraint.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setHeadingConstraint(double set) {
        this.paths.get(paths.size() - 1).setHeadingConstraint(set);
        return this;
    }

    /**
     * This sets the path end t-value (parametric time) constraint on the last Path added to the PathBuilder.
     *
     * @param set This sets the path end t-value (parametric timee) constraint.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setTValueConstraint(double set) {
        this.paths.get(paths.size() - 1).setTValueConstraint(set);
        return this;
    }

    /**
     * This sets the path end timeout constraint on the last Path added to the PathBuilder.
     *
     * @param set This sets the path end timeout constraint.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setTimeoutConstraint(double set) {
        this.paths.get(paths.size() - 1).setTimeoutConstraint(set);
        return this;
    }

    /**
     * This adds a temporal callback on the last Path added to the PathBuilder.
     * This callback is set to run at a specified number of milliseconds after the start of the path.
     *
     * @param time This sets the number of milliseconds of wait between the start of the Path and
     *         the calling of the callback.
     * @param runnable This sets the code for the callback to run. Use lambda statements for this.
     * @return This returns itself with the updated data.
     */
    public PathBuilder addTemporalCallback(double time, Runnable runnable) {
        this.callbacks.add(new FiniteRunAction(new TemporalCallback(paths.size() - 1, time, runnable)));
        return this;
    }

    /**
     * This adds a parametric callback on the last Path added to the PathBuilder.
     * This callback is set to run at a certain point on the Path.
     *
     * @param t This sets the t-value (parametric time) on the Path for when to run the callback.
     * @param runnable This sets the code for the callback to run. Use lambda statements for this.
     * @return This returns itself with the updated data.
     */
    public PathBuilder addParametricCallback(double t, Runnable runnable) {
        this.callbacks.add(new FiniteRunAction(new ParametricCallback(paths.size() - 1, t, follower, runnable)));
        return this;
    }

    /**
     * This adds a pose callback on the last Path added to the PathBuilder.
     * This callback is set to run after the follower crosses the closest point on the path relative to the specified point.
     * @param targetPoint This is the target point relative to which the callback is set.
     * @param runnable This sets the code for the callback to run. Use lambda statements for this.
     * @param initialTValueGuess This should be a decent guess for the t-value of the point on the path closest to the target point. It doesn't need to be very precise, but it'll guide the search and allow for a more accurate computation.
     * @return This returns itself with the updated data.
     */
    public PathBuilder addPoseCallback(Pose targetPoint, Runnable runnable, double initialTValueGuess) {
        this.callbacks.add(new FiniteRunAction(new PoseCallback(follower, paths.size() - 1, targetPoint, runnable, initialTValueGuess, this.paths.get(paths.size() - 1).getCurve())));
        return this;
    }

    /**
     * This adds a callback to the PathChain.
     * @param callback The callback to be added to the PathChain.
     * @return This returns itself with the updated data.
     */
    public PathBuilder addCallback(PathCallback callback) {
        this.callbacks.add(new FiniteRunAction(callback));
        return this;
    }

    /**
     * This adds a callback to the PathChain.
     * @param callback The callback to be added to the PathChain.
     * @param i The maximum number of times to run the action
     * @return This returns itself with the updated data.
     */
    public PathBuilder addCallback(PathCallback callback, int i) {
        this.callbacks.add(new FiniteRunAction(callback, i));
        return this;
    }

    /**
     * This is a condition for a callback to run. This class is functionally the same as a BooleanSupplier.
     * It is used to check if the callback is ready to run.
     */
    public interface CallbackCondition {
        boolean isReady();
    }

    /**
     * This adds a callback to the PathBuilder that will run when a condition is met.
     * This is useful for callbacks that need to run when a certain condition is met, such as a sensor reading.
     *
     * @param condition The condition that must be met for the callback to run.
     * @param action The action to run when the condition is met.
     * @return This returns itself with the updated data.
     */
    public PathBuilder addCallback(CallbackCondition condition, Runnable action) {
        return addCallback(condition, action, 1);
    }

    /**
     * This adds a callback to the PathBuilder that will run when a condition is met.
     * This is useful for callbacks that need to run when a certain condition is met, such as a sensor reading.
     *
     * @param condition The condition that must be met for the callback to run.
     * @param action The action to run when the condition is met.
     * @return This returns itself with the updated data.
     */
    public PathBuilder addCallback(CallbackCondition condition, Runnable action, int i) {
        this.callbacks.add(new FiniteRunAction(new PathCallback() {
            @Override
            public boolean run() {
                action.run();
                return true;
            }

            @Override
            public boolean isReady() {
                return condition.isReady();
            }

            @Override
            public int getPathIndex() {
                return paths.size() - 1; // Assuming the callback is for the last path
            }
        }, i));
        return this;
    }

    /**
     * This adds a callback to the PathBuilder that will run on every loop of the PathChain.
     * This is useful for callbacks that need to run every loop, such as updating telemetry.
     *
     * @param callback The callback to add.
     * @return This returns itself with the updated data.
     */
    public PathBuilder addLoopedCallback(PathCallback callback) {
        this.callbacks.add(callback);
        return this;
    }

    /**
     * This builds all the Path and callback information together into a PathChain.
     * @return This returns a PathChain made of all the specified paths and callbacks.
     */
    public PathChain build() {
        PathChain returnChain = new PathChain(paths);
        returnChain.setCallbacks(callbacks);
        returnChain.setDecelerationType(decelerationType);
        setBrakingStartForAll(constraints.getBrakingStart());
        returnChain.setHeadingInterpolator(headingInterpolator);
        return returnChain;
    }

    /**
     * Sets the PathChain to decelerate based on the entire chain and not only the last path (recommended if the final path is short)
     * @return This returns itself with the updated data.
     */
    public PathBuilder setGlobalDeceleration() {
        this.decelerationType = PathChain.DecelerationType.GLOBAL;
        return this;
    }

    /**
     * Makes this decelerate based on the entire chain and not only the last path (recommended if the last path is short)
     * @param brakingStart sets the BrakingStartMultiplier to the PathConstraints. A lower BrakingStartMultiplier will make the PathChain begin decelerating later, and vice-versa.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setGlobalDeceleration(double brakingStart) {
        this.decelerationType = PathChain.DecelerationType.GLOBAL;
        constraints.setBrakingStart(brakingStart);
        return this;
    }

    /**
     * Sets no deceleration to the PathChain
     * @return This returns itself with the updated data.
     */
    public PathBuilder setNoDeceleration() {
        this.decelerationType = PathChain.DecelerationType.NONE;
        return this;
    }

    /**
     * This sets the constraints to be the default PathBuilder.
     *
     * @param constraints The constraints to set.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setConstraints(PathConstraints constraints) {
        this.constraints = constraints;
        return this;
    }

    /**
     * This sets the constraints for all of the paths.
     *
     * @param constraints The constraints to set.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setConstraintsForAll(PathConstraints constraints) {
        this.constraints = constraints;
        for (Path path : paths) {
            path.setConstraints(constraints);
        }
        return this;
    }

    /**
     * This sets the constraints for the last path.
     *
     * @param constraints The constraints to set.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setConstraintsForLast(PathConstraints constraints) {
        this.constraints = constraints;
        paths.get(paths.size() - 1).setConstraints(constraints);
        return this;
    }

    private void setBrakingStartForAll(double start) {
        for (Path path : paths) path.setBrakingStart(start);
    }
}