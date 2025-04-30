package com.pedropathing.paths;

import com.pedropathing.follower.old.FollowerConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import math.MathFunctions;
import com.pedropathing.util.PathCallback;

import java.util.ArrayList;

/**
 * This is the PathBuilder class. This class makes it easier to create PathChains, so you don't have
 * to individually create Path instances to create a PathChain. A PathBuilder can be accessed
 * through running the pathBuilder() method on an instance of the Follower class, or just creating
 * an instance of PathBuilder regularly.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/11/2024
 */
public class PathBuilder {
    private ArrayList<Path> paths = new ArrayList<>();
    private PathChain.DecelerationType decelerationType = PathChain.DecelerationType.LAST_PATH;
    private ArrayList<PathCallback> callbacks = new ArrayList<>();
    private double decelerationStartMultiplier = FollowerConstants.decelerationStartMultiplier;
    private boolean globalLinearHeadingInterpolation = false;
    private double globalStartHeading = 0;
    private double globalEndHeading = 0;
    private double percentPathStart = 0;
    private boolean reversedLinearInterpol = false;

    /**
     * This is an empty constructor for the PathBuilder class so it can get started.
     * The PathBuilder allows for easier construction of PathChains.
     * The proper usage is using an instance of the Follower class:
     * Follower follower = new Follower(hardwareMap);
     * Then calling "follower.pathBuilder.[INSERT PATH BUILDING METHODS].build();
     * Of course, you can split up the method calls onto separate lines for readability.
     */
    public PathBuilder() {}

    /**
     * This adds a Path to the PathBuilder.
     *
     * @param path The Path being added.
     * @return This returns itself with the updated data.
     */
    public PathBuilder addPath(Path path) {
        this.paths.add(path);
        return this;
    }

    /**
     * This adds a default Path defined by a specified BezierCurve to the PathBuilder.
     *
     * @param curve The curve is turned into a Path and added.
     * @return This returns itself with the updated data.
     */
    public PathBuilder addPath(BezierCurve curve) {
        this.paths.add(new Path(curve));
        return this;
    }


    /**
     * This adds a default Path defined by a specified BezierCurve to the PathBuilder.
     *
     * @param controlPoints This is the specified control points that define the BezierCurve.
     * @return This returns itself with the updated data.
     */
    public PathBuilder addBezierCurve(ArrayList<Pose> controlPoints) {
        return addPath(new BezierCurve(controlPoints));
    }

    /**
     * This adds a default Path defined by a specified BezierLine to the PathBuilder.
     *
     * @param startPoint start point of the line.
     * @param endPoint end point of the line.
     * @return This returns itself with the updated data.
     */
    public PathBuilder addBezierLine(Pose startPoint, Pose endPoint) {
        return addPath(new BezierLine(startPoint, endPoint));
    }

    /**
     * This sets a linear heading interpolation on the last Path added to the PathBuilder.
     *
     * @param startHeading The start of the linear heading interpolation.
     * @param endHeading The end of the linear heading interpolation.
     *         This will be reached at the end of the Path if no end time is specified.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setLinearHeadingInterpolation(double startHeading, double endHeading) {
        this.paths.get(paths.size() - 1).setLinearHeadingInterpolation(startHeading, endHeading);
        return this;
    }

    /**
     * This sets a linear heading interpolation on the last Path added to the PathBuilder.
     *
     * @param startHeading The start of the linear heading interpolation.
     * @param endHeading The end of the linear heading interpolation.
     *         This will be reached at the end of the Path if no end time is specified.
     * @param endTime The end time on the Path that the linear heading interpolation will end.
     *         This value goes from [0, 1] since Bezier curves are parametric functions.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setLinearHeadingInterpolation(double startHeading, double endHeading, double endTime) {
        this.paths.get(paths.size() - 1).setLinearHeadingInterpolation(startHeading, endHeading, endTime);
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
     * This sets a reversed or tangent heading interpolation on the last Path added to the PathBuilder.
     *
     * @param set This sets the heading to reversed tangent following if this parameter is true.
     *         Conversely, this sets a tangent following if this parameter is false.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setReversed(boolean set) {
        this.paths.get(paths.size() - 1).setReversed(set);
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
     * This sets the heading interpolation to custom on the last Path added to the PathBuilder.
     * @param function A function that describes the target heading as a function of t, the parametric variable. Use a lambda expression here.
     */
    public PathBuilder setCustomHeadingInterpolation(Path.CustomHeadingInterpolationFunction function) {
        this.paths.get(paths.size() - 1).setCustomHeadingInterpolation(function);
        return this;
    }

    /**
     * This sets the deceleration multiplier on the last Path added to the PathBuilder.
     *
     * @param set This sets the multiplier for the goal for the deceleration of the robot.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setZeroPowerAccelerationMultiplier(double set) {
        this.paths.get(paths.size() - 1).setZeroPowerAccelerationMultiplier(set);
        return this;
    }

    /**
     * This sets the path end velocity constraint on the last Path added to the PathBuilder.
     *
     * @param set This sets the path end velocity constraint.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setPathEndVelocityConstraint(double set) {
        this.paths.get(paths.size() - 1).setPathEndVelocityConstraint(set);
        return this;
    }

    /**
     * This sets the path end translational constraint on the last Path added to the PathBuilder.
     *
     * @param set This sets the path end translational constraint.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setPathEndTranslationalConstraint(double set) {
        this.paths.get(paths.size() - 1).setPathEndTranslationalConstraint(set);
        return this;
    }

    /**
     * This sets the path end heading constraint on the last Path added to the PathBuilder.
     *
     * @param set This sets the path end heading constraint.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setPathEndHeadingConstraint(double set) {
        this.paths.get(paths.size() - 1).setPathEndHeadingConstraint(set);
        return this;
    }

    /**
     * This sets the path end t-value (parametric time) constraint on the last Path added to the PathBuilder.
     *
     * @param set This sets the path end t-value (parametric time) constraint.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setPathEndTValueConstraint(double set) {
        this.paths.get(paths.size() - 1).setPathEndTValueConstraint(set);
        return this;
    }

    /**
     * This sets the path end timeout constraint on the last Path added to the PathBuilder.
     *
     * @param set This sets the path end timeout constraint.
     * @return This returns itself with the updated data.
     */
    public PathBuilder setPathEndTimeoutConstraint(double set) {
        this.paths.get(paths.size() - 1).setPathEndTimeoutConstraint(set);
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
        this.callbacks.add(new PathCallback(time, runnable, PathCallback.TIME, paths.size() - 1));
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
        this.callbacks.add(new PathCallback(t, runnable, PathCallback.PARAMETRIC, paths.size() - 1));
        return this;
    }

    /**
     * This builds all the Path and callback information together into a PathChain.
     *
     * @return This returns a PathChain made of all the specified paths and callbacks.
     */
    public PathChain build() {
        PathChain returnChain = new PathChain(paths);
        returnChain.setCallbacks(callbacks);
        returnChain.setDecelerationType(decelerationType);
        returnChain.setDecelerationStartMultiplier(decelerationStartMultiplier);

        if (globalLinearHeadingInterpolation) {
            computeGlobalLinearHeading(returnChain);
        }

        return returnChain;
    }

    /**
     * Makes this decelerate based on the entire chain and not only the last path (recommended if the last path is short)
     */
    public PathBuilder setGlobalDeceleration() {
        this.decelerationType = PathChain.DecelerationType.GLOBAL;
        return this;
    }

    /**
     * Makes this decelerate based on the entire chain and not only the last path (recommended if the last path is short)
     */
    public PathBuilder setGlobalDeceleration(double decelerationStartMultiplier) {
        this.decelerationType = PathChain.DecelerationType.GLOBAL;
        this.decelerationStartMultiplier = decelerationStartMultiplier;
        return this;
    }

    /**
     * Sets no deceleration to the pathchain
     */
    public PathBuilder setNoDeceleration() {
        this.decelerationType = PathChain.DecelerationType.NONE;
        return this;
    }

    public PathBuilder setGlobalLinearHeadingInterpolation(double startHeading, double endHeading) {
        return setGlobalLinearHeadingInterpolation(startHeading, endHeading, 0);
    }

    public PathBuilder setGlobalLinearHeadingInterpolation(double startHeading, double endHeading, double percentPathStart) {
        return setGlobalLinearHeadingInterpolation(startHeading, endHeading, percentPathStart, false);
    }

    public PathBuilder setGlobalLinearHeadingInterpolation(double startHeading, double endHeading, double percentPathStart, boolean reversed) {
        globalLinearHeadingInterpolation = true;
        this.percentPathStart = MathFunctions.clamp(percentPathStart, 0, 1);
        this.globalStartHeading = MathFunctions.normalizeAngle(startHeading);
        this.globalEndHeading = MathFunctions.normalizeAngle(endHeading);
        reversedLinearInterpol = reversed;

        return this;
    }

    private void computeGlobalLinearHeading(PathChain pathChain) {
        double turnRadians;

        if (!reversedLinearInterpol) {
            turnRadians = MathFunctions.getSmallestAngleDifference(globalStartHeading, globalEndHeading) * MathFunctions.getTurnDirection(globalStartHeading, globalEndHeading);
        } else {
            turnRadians = -MathFunctions.getSmallestAngleDifference(globalStartHeading, globalEndHeading) * MathFunctions.getTurnDirection(globalStartHeading, globalEndHeading);
        }

        if (turnRadians == 0) {
            for (int i = 0; i < pathChain.size(); i++) {
                pathChain.getPath(i).setConstantHeadingInterpolation(globalStartHeading);
            }

            return;
        }

        double sumLength = 0;
        int index;

        for (index = pathChain.size() - 1; index >= 0 && sumLength < pathChain.length() * percentPathStart; index--) {
            sumLength += pathChain.getPath(index).length();
        }

        double firstPathLength = pathChain.getPath(index).length();
        double firstPercentage = 1-(sumLength - pathChain.length() * percentPathStart - firstPathLength)/firstPathLength;
        double firstPathHeadingDelta = firstPathLength / pathChain.length() / percentPathStart * turnRadians;
        pathChain.getPath(index).setLinearHeadingInterpolation(globalStartHeading, globalStartHeading + firstPathHeadingDelta, firstPercentage);

        double turnedRadians = firstPathHeadingDelta;
        for (int i = 0; i < pathChain.size(); i++) {
            if (i < index) {
                pathChain.getPath(i).setConstantHeadingInterpolation(globalStartHeading);
            } else if (i > index) {
                Path path = pathChain.getPath(i);
                double pathDelta = path.length() / pathChain.length() * turnRadians;
                path.setLinearHeadingInterpolation(globalStartHeading + turnedRadians, globalStartHeading + turnedRadians + pathDelta);
                turnedRadians += pathDelta;
            }
        }
    }
}