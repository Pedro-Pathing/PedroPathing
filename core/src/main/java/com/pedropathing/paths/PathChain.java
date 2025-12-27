package com.pedropathing.paths;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.callbacks.ParametricCallback;
import com.pedropathing.paths.callbacks.PathCallback;
import com.pedropathing.paths.callbacks.TemporalCallback;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;

/**
 * This is the PathChain class. This class handles chaining together multiple Paths into a larger
 * collection of Paths that can be run continuously. Additionally, this allows for the addition of
 * PathCallbacks to specific Paths in the PathChain, allowing for non-blocking code to be run in
 * the middle of a PathChain.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Havish Sripada - 12808 RevAmped Robotics
 * @version 1.0, 3/11/2024
 */
public class PathChain {
    private ArrayList<Path> pathChain = new ArrayList<>();
    private double length = 0;

    public enum DecelerationType {
        NONE,
        GLOBAL,
        LAST_PATH
    }
    private DecelerationType decelerationType = DecelerationType.LAST_PATH;
    private ArrayList<PathCallback> callbacks = new ArrayList<>();
    public HeadingInterpolator headingInterpolator = null;
    private Double closestPointHeadingGoal;
    private Double finalHeadingGoal;

    /**
     * This creates a new PathChain from some specified Paths.
     *
     * IMPORTANT NOTE: Order matters here. The order in which the Paths are input is the order in
     * which they will be run.
     *
     * @param paths the specified Paths.
     */
    public PathChain(Path... paths) {
        this(PathConstraints.defaultConstraints, paths);
    }

    /**
     * This creates a new PathChain from some specified Paths and a PathConstraints.
     *
     * IMPORTANT NOTE: Order matters here. The order in which the Paths are input is the order in
     * which they will be run.
     *
     * @param paths the specified Paths.
     * @param constraints the PathConstraints for the PathChain.
     */
    public PathChain(PathConstraints constraints, Path... paths) {
        for (Path path : paths) {
            path.setConstraints(constraints);
            pathChain.add(path);
            length += path.length();
        }
    }

    /**
     * This creates a new PathChain from an ArrayList of Paths.
     *
     * IMPORTANT NOTE: Order matters here. The order in which the Paths are input is the order in
     * which they will be run.
     *
     * @param paths the ArrayList of Paths.
     */
    public PathChain(ArrayList<Path> paths) {
        this(PathConstraints.defaultConstraints, paths);
    }

    /**
     * This creates a new PathChain from an ArrayList of Paths.
     *
     * IMPORTANT NOTE: Order matters here. The order in which the Paths are input is the order in
     * which they will be run.
     *
     * @param paths the ArrayList of Paths.
     * @param constraints the PathConstraints for the PathChain.
     */
    public PathChain(PathConstraints constraints, ArrayList<Path> paths) {
        for (Path path : paths) {
            path.setConstraints(constraints);
            length += path.length();
        }

        pathChain = paths;
    }

    /**
     * This returns the Path on the PathChain at a specified index.
     *
     * @param index the index.
     * @return returns the Path at the index.
     */
    public Path getPath(int index) {
        return pathChain.get(index);
    }

    /**
     * This returns the size of the PathChain.
     *
     * @return returns the size of the PathChain.
     */
    public int size() {
        return pathChain.size();
    }

    /**
     * This sets the PathCallbacks of the PathChain with some specified PathCallbacks.
     *
     * @param callbacks the specified PathCallbacks.
     */
    public void setCallbacks(PathCallback... callbacks) {
        this.callbacks.addAll(Arrays.asList(callbacks));
    }

    /**
     * This sets the PathCallbacks of the PathChain with an ArrayList of PathCallbacks.
     *
     * @param callbacks the ArrayList of PathCallbacks.
     */
    public void setCallbacks(ArrayList<PathCallback> callbacks) {
        Comparator<PathCallback> callbackComparator = (pathCallback, other) -> {
            if (pathCallback instanceof ParametricCallback && other instanceof ParametricCallback)
                return Double.compare(((ParametricCallback) pathCallback).getStartCondition(), ((ParametricCallback) other).getStartCondition());
            if (pathCallback instanceof TemporalCallback && other instanceof TemporalCallback)
                return Double.compare(((TemporalCallback) pathCallback).getStartCondition(), ((TemporalCallback) other).getStartCondition());
            return 0;
        };
        Collections.sort(callbacks, callbackComparator);
        this.callbacks = callbacks;
    }

    /**
     * This returns the PathCallbacks of this PathChain in an ArrayList.
     *
     * @return returns the PathCallbacks.
     */
    public ArrayList<PathCallback> getCallbacks() {
        return callbacks;
    }

    /**
     * This resets the PathCallbacks of this PathChain.
     */

    public void resetCallbacks() {
        for (PathCallback callback : callbacks) {
            callback.reset();
        }
    }

    /**
     * This sets the deceleration type of the PathChain.
     * @param decelerationType the deceleration type to set
     */
    public void setDecelerationType(DecelerationType decelerationType) {
        this.decelerationType = decelerationType;
    }

    /**
     * This returns the deceleration type of the PathChain.
     * @return the deceleration type of the PathChain
     */
    public DecelerationType getDecelerationType() {
        return decelerationType;
    }

    /**
     * This returns the length of the PathChain.
     * @return the length of the PathChain
     */
    public double length() {
        return length;
    }

    /**
     * This returns the end pose of the PathChain.
     * @return the end pose of the PathChain
     */
    public Pose endPose() {
        Path last = pathChain.get(pathChain.size() - 1);
        return last.endPose();
    }

    /**
     * This returns the end point of the PathChain.
     * @return the end point of the PathChain
     */
    public Pose endPoint() {
        Path last = pathChain.get(pathChain.size() - 1);
        return last.getLastControlPoint();
    }

    /**
     * This sets the constraints for all Paths in the PathChain.
     * @param constraints the constraints to set
     */
    public void setConstraintsForAll(PathConstraints constraints) {
        for (Path path : pathChain) {
            path.setConstraints(constraints);
        }
    }

    /**
     * Represents a specific point within a {@link PathChain}, defined by a path index and a t-value (parametric position).
     * Provides utility methods to access the corresponding path, pose, point, tangent vector, and heading goal.
     *
     * @author Havish Sripada - 12808 RevAmped Robotics
     */
    public static class PathT {
        private final int pathIndex;
        private final double t;

        public PathT(int pathIndex, double t) {
            this.pathIndex = pathIndex;
            this.t = t;
        }

        public int pathIndex() {
            return pathIndex;
        }

        public double t() {
            return t;
        }

        public Path getPath(PathChain pathChain) {
            return pathChain.getPath(pathIndex);
        }

        public Pose getPose(PathChain pathChain) {
            return pathChain.getPath(pathIndex).getPose(t);
        }

        public Pose getPoint(PathChain pathChain) {
            return pathChain.getPath(pathIndex).getPoint(t);
        }

        public Vector getTangentVector(PathChain pathChain) {
            return pathChain.getPath(pathIndex).getTangentVector(t);
        }

        public double getHeadingGoal(PathChain pathChain) {
            return pathChain.getPath(pathIndex).getHeadingGoal(t);
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (!(o instanceof PathT)) return false;
            PathT other = (PathT) o;
            return pathIndex == other.pathIndex && Double.compare(other.t, t) == 0;
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(pathIndex, t);
        }

        @Override
        public String toString() {
            return "PathT[pathIndex=" + pathIndex + ", t=" + t + "]";
        }
    }


    /**
     * This gets the path that corresponds to the given completion amount of the chain
     * @param t completion of the PathChain from [0,1] based on distance traveled
     */
    private PathT chainCompletionToPath(double t) {
        double lengthSum = 0;
        double currentT = 1;
        for (int i = 0; i < pathChain.size(); i++) {
            lengthSum += pathChain.get(i).length();

            if (lengthSum > length) {
                return new PathT(i, pathChain.get(i).getTFromPathCompletion(currentT));
            }

            currentT = t - lengthSum / length;
        }

        return new PathT(pathChain.size()-1, pathChain.get(pathChain.size() - 1).getTFromPathCompletion(currentT));
    }

    /**
     * Sets the path's heading interpolation
     * @param headingInterpolator the heading interpolation to set
     */
    public void setHeadingInterpolator(HeadingInterpolator headingInterpolator) {
        this.headingInterpolator = headingInterpolator;
    }

    /**
     * This returns the heading goal of the path.
     * @param pathTValue this is the path and t-value
     * @return the heading goal of the path
     */
    public double getHeadingGoal(PathT pathTValue) {
        if (headingInterpolator != null) {
            double sumLength = 0;

            for (int i = 0; i < pathTValue.pathIndex(); i++) {
                sumLength += pathChain.get(i).length();
            }

            double pathInitialTValue = sumLength / length;
            double chainT = pathInitialTValue + pathChain.get(pathTValue.pathIndex()).getDistanceTraveled(pathTValue.t()) / length;
            return headingInterpolator.interpolate(new PathPoint(chainT, pathTValue.getPoint(this), pathTValue.getTangentVector(this)));
        }

        return pathTValue.getHeadingGoal(this);
    }

    /**
     * This returns the heading goal of the path.
     * @param pathTValue this is the path and t-value
     * @return the heading goal of the path
     */
    public double getClosestPointHeadingGoal(PathT pathTValue) {
        if (closestPointHeadingGoal == null) {
            closestPointHeadingGoal = getHeadingGoal(pathTValue);
        }

        return closestPointHeadingGoal;
    }

    /**
     * This returns the tangent vector of the point in the path.
     * @param pathTValue this is the path and t-value
     * @return the tangent vector of the point in the path
     */
    public Vector getTangentVector(PathT pathTValue) {
        return pathTValue.getTangentVector(this);
    }

    /**
     * This returns the pose of the point in the path.
     * @param pathTValue this is the path and t-value
     * @return the pose of the point in the path
     */
    public Pose getPose(PathT pathTValue) {
        return pathTValue.getPose(this);
    }

    /**
     * This returns the point in the path, without a heading goal.
     * @param pathTValue this is the path and t-value
     * @return the point in the path
     */
    public Pose getPoint(PathT pathTValue) {
        return pathTValue.getPoint(this);
    }

    /**
     * This returns the pose information of the point in the path (with the heading goal).
     * @param pathTValue this is the path and t-value
     * @return the pose information of the point in the path
     */
    public PathPoint getPoseInformation(PathT pathTValue) {
        return new PathPoint(pathTValue.t(), pathTValue.getPose(this), pathTValue.getTangentVector(this));
    }

    /**
     * This returns the last Path in the PathChain.
     * @return the last Path in the PathChain
     */
    public Path lastPath() {
        return pathChain.get(pathChain.size() - 1);
    }

    /**
     * This returns the first Path in the PathChain.
     * @return the first Path in the PathChain
     */
    public Path firstPath() {
        if (pathChain.isEmpty()) {
            return null;
        }
        return pathChain.get(0);
    }

    public Double getFinalHeadingGoal() {
        if (finalHeadingGoal == null) {
            if (headingInterpolator == null) finalHeadingGoal = lastPath().getHeadingGoal(1);
            else finalHeadingGoal = headingInterpolator.interpolate(new PathPoint(1, lastPath().endPose(), lastPath().getEndTangent()));
        }

        return finalHeadingGoal;
    }

    public void update() {
        closestPointHeadingGoal = null;
    }
}
