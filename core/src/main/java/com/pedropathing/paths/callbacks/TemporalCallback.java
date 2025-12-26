package com.pedropathing.paths.callbacks;

import java.util.concurrent.TimeUnit;

/**
 * This is the TemporalCallback class. This class handles callbacks that occur a given amount of time after a Path in a PathChain has begun running.
 *
 * @author Havish Sripada - 12808 RevAmped Robotics
 */
public class TemporalCallback implements PathCallback {
    /**
     * The time (in milliseconds) after which the callback should be triggered since initialization.
     */
    private final double startCondition;

    /**
     * The action to execute when the callback is triggered.
     */
    private final Runnable runnable;

    /**
     * The start time (in milliseconds) when the callback was initialized.
     */
    private double startTime = 0;
    /**
     * The index of the path this callback is associated with.
     */
    private int index;

    /**
     * This creates a new TemporalCallback from an index, start condition, and runnable.
     * @param index the index of the path in the path chain
     * @param startCondition the time since the start of the path to run the callback
     * @param runnable the runnable to run
     */
    public TemporalCallback(int index, double startCondition, Runnable runnable) {
        this.index = index;
        this.startCondition = startCondition;
        this.runnable = runnable;
    }
    
    /**
     * Executes the callback action.
     * @return true after running the action
     */
    @Override
    public boolean run() {
        runnable.run();
        return true;
    }

    /**
     * This checks if the callback is ready to run.
     * @return true if the callback is ready to run
     */
    @Override
    public boolean isReady() {
        return TimeUnit.NANOSECONDS.convert(System.nanoTime(), TimeUnit.MILLISECONDS) - startTime >= startCondition;
    }
    
    /**
     * Initializes the callback by recording the current time.
     */
    @Override
    public void initialize() {
        startTime = TimeUnit.NANOSECONDS.convert(System.nanoTime(), TimeUnit.MILLISECONDS);
    }

    /**
     * Returns the index of the path this callback is associated with.
     * @return the path index
     */
    @Override
    public int getPathIndex() {
        return index;
    }

    public double getStartCondition() {
        return startCondition;
    }
}
