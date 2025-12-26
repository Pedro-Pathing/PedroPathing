package com.pedropathing.paths.callbacks;

import com.pedropathing.follower.Follower;

/**
 * This is the ParametricCallback class. This class handles callbacks that occur after a certain t-value of the Path in a PathChain has begun achieved.
 *
 * @author Havish Sripada - 12808 RevAmped Robotics
 */
public class ParametricCallback implements PathCallback {
    private final Follower follower;
    private final double startCondition;
    private final Runnable runnable;
    private final int index;

    /**
     * Constructor for the ParametricCallback.
     * @param index The index of the path that this callback is for
     * @param startCondition The path completion at which the callback should occur
     * @param follower The follower that is running the path
     * @param runnable The runnable that will be run when the callback is triggered
     */
    public ParametricCallback(int index, double startCondition, Follower follower, Runnable runnable) {
        this.index = index;
        this.follower = follower;
        this.startCondition = startCondition;
        this.runnable = runnable;
    }

    /**
     * This method runs the callback.
     * @return true if the action was successful
     */
    @Override
    public boolean run() {
        runnable.run();
        return true;
    }

    /**
     * This method checks if the callback is ready to run.
     * @return if the callback is ready to run
     */
    @Override
    public boolean isReady() {
        return follower.atParametricEnd() || follower.getPathCompletion() >= startCondition;
    }

    /**
     * This method returns the index of the path that this callback is for.
     * @return the index of the path that this callback is for
     */
    @Override
    public int getPathIndex() {
        return index;
    }

    public double getStartCondition() {
        return startCondition;
    }
}
