package com.pedropathing.paths;

import com.pedropathing.follower.old.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.PathCallback;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * This is the PathChain class. This class handles chaining together multiple Paths into a larger
 * collection of Paths that can be run continuously. Additionally, this allows for the addition of
 * PathCallbacks to specific Paths in the PathChain, allowing for non-blocking code to be run in
 * the middle of a PathChain.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
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
    private double decelerationStartMultiplier = FollowerConstants.decelerationStartMultiplier;
    private ArrayList<PathCallback> callbacks = new ArrayList<>();

    /**
     * This creates a new PathChain from some specified Paths.
     *
     * IMPORTANT NOTE: Order matters here. The order in which the Paths are input is the order in
     * which they will be run.
     *
     * @param paths the specified Paths.
     */
    public PathChain(Path... paths) {
        for (Path path : paths) {
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

    public void resetCallbacks() {
        for (PathCallback callback : callbacks) {
            callback.reset();
        }
    }

    public void setDecelerationType(DecelerationType decelerationType) {
        this.decelerationType = decelerationType;
    }

    public DecelerationType getDecelerationType() {
        return decelerationType;
    }

    public double length() {
        return length;
    }

    public void setDecelerationStartMultiplier(double decelerationStartMultiplier) {
        this.decelerationStartMultiplier = decelerationStartMultiplier;
    }

    public double getDecelerationStartMultiplier() {
        return decelerationStartMultiplier;
    }

    public Pose endPose() {
        Path last = pathChain.get(pathChain.size() - 1);
        return last.endPose();
    }

    public Pose endPoint() {
        Path last = pathChain.get(pathChain.size() - 1);
        return last.getLastControlPoint();
    }
}
